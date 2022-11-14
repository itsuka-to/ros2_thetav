/*
  Copyright 2020 K. Takeo. All rights reserved.
  Copyright 2020 Tomoya Itsuka (@itsuka-to)

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  3. Neither the name of the author nor other contributors may be
  used to endorse or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

 */
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include "libuvc/libuvc.h"
#include "thetauvc.h"

#define MAX_PIPELINE_LEN 8096 //1024

using namespace std::chrono_literals;

struct gst_src {
	GstElement *pipeline;
	GstElement *appsrc;

	GMainLoop *loop;
	GTimer *timer;
	guint framecount;
	guint id;
	guint bus_watch_id;
	uint32_t dwFrameInterval;
	uint32_t dwClockFrequency;
};

struct gst_src src;

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image, std::allocator<void>>> image_pub;

static GstFlowReturn
publish_image (GstMapInfo map)
{
    int dataLength;
    guint8 *rdata;

    dataLength = map.size;
    rdata = map.data;

    sensor_msgs::msg::Image image;
    image.header.stamp = rclcpp::Clock().now();
    image.width = 1920;
    image.height = 960;
    // image.width = 3840;
    // image.height = 1920;
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = dataLength;

    std::vector<unsigned char> values(rdata, (unsigned char *)rdata + dataLength);
    image.data = values;
    image_pub->publish(image);
}

static gboolean
gst_bus_cb(GstBus *bus, GstMessage *message, gpointer data)
{
	GError *err;
	gchar *dbg;

	switch (GST_MESSAGE_TYPE(message)) {
	case GST_MESSAGE_ERROR:
		gst_message_parse_error(message, &err, &dbg);
		// g_print("Error: %s %s\n", GST_OBJECT_NAME(message->src), err->message);
		// g_print("Debugging info: %s\n", (dbg) ? dbg : "none");
		std::cout << "Error: " << GST_OBJECT_NAME(message->src) << err->message << std::endl;
        std::cout << "Debugging info: " << (dbg ? dbg : "none") << std::endl;
        g_error_free(err);
		g_free(dbg);
		g_main_loop_quit(src.loop);
		break;
	case GST_MESSAGE_INFO:
		gst_message_parse_info(message, &err, &dbg);
		g_print("Info: %s %s\n", GST_OBJECT_NAME(message->src), err->message);
		g_print("more info: %s\n", (dbg) ? dbg : "none");
		g_error_free(err);
		g_free(dbg);
	default:
		break;
	}

	return TRUE;
}

GstFlowReturn cb_new_sample(GstAppSink *sink, gpointer data)
{
    static int counter = 0;
    GstBuffer *app_buffer, *buffer;
    GstSample *sample;
    GstMapInfo map;

    sample = gst_app_sink_pull_sample(sink);
    buffer = gst_sample_get_buffer(sample);
    app_buffer = gst_buffer_copy_deep(buffer);
    gst_buffer_map(app_buffer, &map, GST_MAP_WRITE);
    publish_image(map);

    gst_sample_unref (sample);
    gst_buffer_unmap (app_buffer, &map);
    gst_buffer_unref(app_buffer);
    if (sample == NULL) {
        return GST_FLOW_EOS;
    } else {
        return GST_FLOW_OK;
    }
}



int
gst_src_init(int *argc, char ***argv, char *pipeline)
{
	GstCaps *caps;
	GstBus *bus;
    GstElement *appsink;
	char pipeline_str[MAX_PIPELINE_LEN];

	snprintf(pipeline_str, MAX_PIPELINE_LEN, "appsrc name=ap ! queue ! h264parse ! queue ! %s ", pipeline);

	gst_init(argc, argv);
	src.timer = g_timer_new();
	src.loop = g_main_loop_new(NULL, TRUE);
	src.pipeline = gst_parse_launch(pipeline_str, NULL);

	g_assert(src.pipeline);
	if (src.pipeline == NULL)
		return FALSE;
	gst_pipeline_set_clock(GST_PIPELINE(src.pipeline), gst_system_clock_obtain());

	src.appsrc = gst_bin_get_by_name(GST_BIN(src.pipeline), "ap");

	caps = gst_caps_new_simple("video/x-h264",
		"framerate", GST_TYPE_FRACTION, 30000, 1001,
		"stream-format", G_TYPE_STRING, "byte-stream",
		"profile", G_TYPE_STRING, "constrained-baseline", NULL);
	gst_app_src_set_caps(GST_APP_SRC(src.appsrc), caps);

	bus = gst_pipeline_get_bus(GST_PIPELINE(src.pipeline));
	src.bus_watch_id = gst_bus_add_watch(bus, gst_bus_cb, NULL);
	gst_object_unref(bus);

    appsink = gst_bin_get_by_name(GST_BIN(src.pipeline), "appsink");
    if (appsink == NULL)
    {
        g_print("appsink is NULL\n"); 
    }
    g_signal_connect(appsink, "new-sample", G_CALLBACK(cb_new_sample), NULL);
    gst_object_unref(appsink);

	return TRUE;
}


void cb(uvc_frame_t *frame, void *ptr)
{
	struct gst_src *s;
	GstBuffer *buffer;
	GstFlowReturn ret;
	GstMapInfo map;
	gdouble ms;
	uint32_t pts;

	s = (struct gst_src *)ptr;
	ms = g_timer_elapsed(s->timer, NULL);

	buffer = gst_buffer_new_allocate(NULL, frame->data_bytes, NULL);;
	GST_BUFFER_PTS(buffer) = frame->sequence * s->dwFrameInterval*100;
	GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
	GST_BUFFER_DURATION(buffer) = s->dwFrameInterval*100;
	GST_BUFFER_OFFSET(buffer) = frame->sequence;
	s->framecount++;

	gst_buffer_map(buffer, &map, GST_MAP_WRITE);
	memcpy(map.data, frame->data, frame->data_bytes);
	gst_buffer_unmap(buffer, &map);

	g_signal_emit_by_name(s->appsrc, "push-buffer", buffer, &ret);
	gst_buffer_unref(buffer);

	if (ret != GST_FLOW_OK)
		fprintf(stderr, "pushbuffer errorn");
	return;
}



int main(int argc, char * argv[])
{

    // uvc
    uvc_context_t *ctx;
    uvc_device_t *dev;
	uvc_device_t **devlist;
	uvc_device_handle_t *devh;
	uvc_stream_ctrl_t ctrl;
    uvc_error_t res;

    struct gst_src *s;
	int idx;
	char *pipe_proc;
	char *cmd_name;

    // pipe_proc = " decodebin ! autovideosink sync=false";  // x-window view
    pipe_proc = " decodebin ! queue ! videoconvert n_threads=8 ! queue ! video/x-raw,format=RGB ! appsink name=appsink emit-signals=true";
	if (!gst_src_init(&argc, &argv, pipe_proc))
		return -1;
    res = uvc_init((uvc_context_t **)&ctx, (libusb_context *)NULL);
    if(res != UVC_SUCCESS){
        uvc_perror(res, "uvc_init");
        return res;
    }

    res = thetauvc_find_device(ctx, &dev, 0);
	if (res != UVC_SUCCESS) {
		fprintf(stderr, "THETA not found\n");
		uvc_exit(ctx);
        return res;
	}

	res = uvc_open(dev, &devh);
	if (res != UVC_SUCCESS) {
		fprintf(stderr, "Can't open THETA\n");
		uvc_exit(ctx);
        return res;
	}



	gst_element_set_state(src.pipeline, GST_STATE_PLAYING);

	// res = thetauvc_get_stream_ctrl_format_size(devh,
	// 		THETAUVC_MODE_UHD_2997, &ctrl);
	res = thetauvc_get_stream_ctrl_format_size(devh,
			THETAUVC_MODE_FHD_2997, &ctrl);
	src.dwFrameInterval = ctrl.dwFrameInterval;
	src.dwClockFrequency = ctrl.dwClockFrequency;

    // std::cout << ctrl << std::endl;
    
    rclcpp::init(argc, argv);
    rclcpp::QoS custom_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    auto node = rclcpp::Node::make_shared("thetav_publisher");
    image_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "thetav", 
        custom_qos
    );
    auto publish_count = 0;
    rclcpp::WallRate loop_rate(500ms);

    res = uvc_start_streaming(devh, &ctrl, cb, &src, 0);
    while (rclcpp::ok() && (res == UVC_SUCCESS)) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    gst_element_set_state(src.pipeline, GST_STATE_NULL);
    g_source_remove(src.bus_watch_id);
    uvc_stop_streaming(devh);

    uvc_close(devh);
    uvc_exit(ctx);
    rclcpp::shutdown();
    return 0;
}