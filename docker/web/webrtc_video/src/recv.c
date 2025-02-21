#include <gstreamer-1.0/gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jansson.h>
#include <libwebsockets.h>
#include <pthread.h>

void handle_sdp(json_t *sdp);

#define SIGNALING_SERVER "ws://87.119.173.184:8765"

// Global variables
struct lws_context *context;
struct lws *wsi;
GstElement *pipeline = NULL, *webrtcbin = NULL, *compositor = NULL;
GstPromise *promise = NULL;

// WebSocket callbacks and WebSocket context
static int callback_websockets(struct lws *wsi, enum lws_callback_reasons reason,
                                void *user, void *in, size_t len) {
    switch (reason) {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            printf("Connected to signaling server\n");
            break;
        case LWS_CALLBACK_CLIENT_RECEIVE:
            // Handle WebSocket messages here (SDP, ICE candidates)
            {
                json_error_t error;
                json_t *msg = json_loads(in, 0, &error);
                if (!msg) {
                    fprintf(stderr, "Error parsing JSON: %s\n", error.text);
                    return -1;
                }

                if (json_is_object(msg)) {
                    json_t *sdp = json_object_get(msg, "sdp");
                    if (sdp) {
                        handle_sdp(sdp);
                    }
                }
                json_decref(msg);
            }
            break;
        case LWS_CALLBACK_CLIENT_WRITEABLE:
            // Handle outgoing messages here (SDP, ICE candidates)
            break;
        case LWS_CALLBACK_CLOSED:
            printf("WebSocket closed\n");
            break;
        default:
            break;
    }
    return 0;
}

struct lws_protocols protocols[] = {
    {
        .name = "websocket",
        .callback = callback_websockets,
        .per_session_data_size = 0,
        .rx_buffer_size = 0,
    },
    { NULL, NULL, 0, 0 } /* Terminator */
};

// Function to connect to signaling server via WebSocket
int connect_websocket() {
    struct lws_client_connect_info i = { 0 };
    i.context = context;
    i.address = "87.119.173.184";
    i.port = 8765;
    i.ssl_connection = 0;
    i.protocol = protocols[0].name;

    wsi = lws_client_connect_via_info(&i);
    if (!wsi) {
        printf("WebSocket connection failed\n");
        return -1;
    }

    return 0;
}

// SDP handling function
void handle_sdp(json_t *sdp) {
    const char *sdp_str = json_string_value(json_object_get(sdp, "sdp"));
    printf("Received SDP offer:\n%s\n", sdp_str);

    GstSDPMessage *sdp_msg = NULL;
    gst_sdp_message_parse_buffer((guint8 *)sdp_str, strlen(sdp_str), &sdp_msg);
    offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_msg);

    // Set remote description
    promise = gst_promise_new_with_change_func(on_answer_created, NULL, NULL);
    gst_element_emit_by_name(webrtcbin, "set-remote-description", offer, promise);

    // Create and send answer
    gst_element_emit_by_name(webrtcbin, "create-answer", NULL, promise);
}

// Callback function for handling the SDP answer creation
void on_answer_created(GstPromise *promise, gpointer user_data) {
    GstWebRTCSessionDescription *answer;
    GstSDPMessage *sdp_msg;

    gst_promise_wait(promise);
    gst_promise_unref(promise);

    answer = (GstWebRTCSessionDescription *)gst_promise_get_reply(promise);
    gst_sdp_message_parse_buffer(answer->sdp->data, answer->sdp->len, &sdp_msg);

    // Send SDP answer to peer
    char *answer_text = gst_sdp_message_as_text(sdp_msg);
    json_t *msg = json_object();
    json_object_set_new(msg, "sdp", json_string(answer_text));

    // Send via WebSocket
    const char *out_msg = json_dumps(msg, 0);
    lws_write(wsi, (unsigned char *)out_msg, strlen(out_msg), LWS_WRITE_TEXT);
    free(out_msg);
    json_decref(msg);
}

void setup_pipeline() {
    GstElement *compositor = gst_element_factory_make("compositor", "compositor");
    if (!compositor) {
        printf("ERROR: Could not create compositor.\n");
        return;
    }
    gst_bin_add(GST_BIN(pipeline), compositor);
}

int main() {
    // Initialize GStreamer
    gst_init(NULL, NULL);

    // Create GStreamer pipeline
    pipeline = gst_pipeline_new("pipeline");
    setup_pipeline();

    // Initialize WebSocket
    struct lws_context_creation_info info = { 0 };
    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = protocols;

    context = lws_create_context(&info);
    if (!context) {
        printf("WebSocket context creation failed\n");
        return -1;
    }

    // Connect to signaling server
    if (connect_websocket() < 0) {
        lws_context_destroy(context);
        return -1;
    }

    // Main loop for WebSocket and GStreamer processing
    while (1) {
        lws_service(context, 0);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    // Cleanup
    lws_context_destroy(context);
    gst_object_unref(pipeline);

    return 0;
}
