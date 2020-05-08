/**
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <string.h>
#include <string>
#include <sstream>
#include <iostream>
#include <ostream>
#include <fstream>
#include "ros2_deepstream/gst_ds_rclcpp_publisher.h"
#include <npp.h>
#include <sys/time.h>
GST_DEBUG_CATEGORY_STATIC (gst_ds_rclcpp_publisher_debug);
#define GST_CAT_DEFAULT gst_ds_rclcpp_publisher_debug
#define USE_EGLIMAGE 1

static GQuark _dsmeta_quark = 0;

/* Enum to identify properties */
enum
{
  PROP_0,
  PROP_UNIQUE_ID,
  PROP_PROCESSING_WIDTH,
  PROP_PROCESSING_HEIGHT,
  PROP_PROCESS_FULL_FRAME,
  PROP_GPU_DEVICE_ID,
  PROP_NODE_NAME,
  PROP_TOPIC_NAME,
};

#define CHECK_NVDS_MEMORY_AND_GPUID(object, surface)  \
  ({ int _errtype=0;\
   do {  \
    if ((surface->memType == NVBUF_MEM_DEFAULT || surface->memType == NVBUF_MEM_CUDA_DEVICE) && \
        (surface->gpuId != object->gpu_id))  { \
    GST_ELEMENT_ERROR (object, RESOURCE, FAILED, \
        ("Input surface gpu-id doesnt match with configured gpu-id for element," \
         " please allocate input using unified memory, or use same gpu-ids"),\
        ("surface-gpu-id=%d,%s-gpu-id=%d",surface->gpuId,GST_ELEMENT_NAME(object),\
         object->gpu_id)); \
    _errtype = 1;\
    } \
    } while(0); \
    _errtype; \
  })


/* Default values for properties */
#define DEFAULT_UNIQUE_ID 15
#define DEFAULT_PROCESSING_WIDTH 640
#define DEFAULT_PROCESSING_HEIGHT 368
#define DEFAULT_PROCESS_FULL_FRAME TRUE
#define DEFAULT_GPU_ID 0

#define RGB_BYTES_PER_PIXEL 3
#define RGBA_BYTES_PER_PIXEL 4
#define Y_BYTES_PER_PIXEL 1
#define UV_BYTES_PER_PIXEL 2

#define MIN_INPUT_OBJECT_WIDTH 16
#define MIN_INPUT_OBJECT_HEIGHT 16

#define CHECK_NPP_STATUS(npp_status,error_str) do { \
  if ((npp_status) != NPP_SUCCESS) { \
    g_print ("Error: %s in %s at line %d: NPP Error %d\n", \
        error_str, __FILE__, __LINE__, npp_status); \
    goto error; \
  } \
} while (0)

#define CHECK_CUDA_STATUS(cuda_status,error_str) do { \
  if ((cuda_status) != cudaSuccess) { \
    g_print ("Error: %s in %s at line %d (%s)\n", \
        error_str, __FILE__, __LINE__, cudaGetErrorName(cuda_status)); \
    goto error; \
  } \
} while (0)

/* By default NVIDIA Hardware allocated memory flows through the pipeline. We
 * will be processing on this type of memory only. */
#define GST_CAPS_FEATURE_MEMORY_NVMM "memory:NVMM"
static GstStaticPadTemplate gst_ds_rclcpp_publisher_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_NVMM,
            "{ NV12, RGBA, I420 }")));

static GstStaticPadTemplate gst_ds_rclcpp_publisher_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE_WITH_FEATURES
        (GST_CAPS_FEATURE_MEMORY_NVMM,
            "{ NV12, RGBA, I420 }")));

/* Define our element type. Standard GObject/GStreamer boilerplate stuff */
#define gst_ds_rclcpp_publisher_parent_class parent_class
G_DEFINE_TYPE (GstDsRclcppPublisher, gst_ds_rclcpp_publisher, GST_TYPE_BASE_TRANSFORM);

static void gst_ds_rclcpp_publisher_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_ds_rclcpp_publisher_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_ds_rclcpp_publisher_set_caps (GstBaseTransform * btrans,
    GstCaps * incaps, GstCaps * outcaps);
static gboolean gst_ds_rclcpp_publisher_start (GstBaseTransform * btrans);
static gboolean gst_ds_rclcpp_publisher_stop (GstBaseTransform * btrans);

static GstFlowReturn gst_ds_rclcpp_publisher_transform_ip (GstBaseTransform *
    btrans, GstBuffer * inbuf);

static void
attach_metadata_full_frame (GstDsRclcppPublisher * ds_rclcpp_publisher, NvDsFrameMeta *data,
    gdouble scale_ratio, guint batch_id);
static void attach_metadata_object (GstDsRclcppPublisher * ds_rclcpp_publisher,
    NvDsObjectMeta * obj_meta);

/* Install properties, set sink and src pad capabilities, override the required
 * functions of the base class, These are common to all instances of the
 * element.
 */
static void
gst_ds_rclcpp_publisher_class_init (GstDsRclcppPublisherClass * klass)
{
  GObjectClass *gobject_class;
  GstElementClass *gstelement_class;
  GstBaseTransformClass *gstbasetransform_class;

  // Indicates we want to use DS buf api
  g_setenv ("DS_NEW_BUFAPI", "1", TRUE);

  gobject_class = (GObjectClass *) klass;
  gstelement_class = (GstElementClass *) klass;
  gstbasetransform_class = (GstBaseTransformClass *) klass;

  /* Overide base class functions */
  gobject_class->set_property = GST_DEBUG_FUNCPTR (gst_ds_rclcpp_publisher_set_property);
  gobject_class->get_property = GST_DEBUG_FUNCPTR (gst_ds_rclcpp_publisher_get_property);

  gstbasetransform_class->set_caps = GST_DEBUG_FUNCPTR (gst_ds_rclcpp_publisher_set_caps);
  gstbasetransform_class->start = GST_DEBUG_FUNCPTR (gst_ds_rclcpp_publisher_start);
  gstbasetransform_class->stop = GST_DEBUG_FUNCPTR (gst_ds_rclcpp_publisher_stop);

  gstbasetransform_class->transform_ip =
      GST_DEBUG_FUNCPTR (gst_ds_rclcpp_publisher_transform_ip);

  /* Install properties */
  g_object_class_install_property (gobject_class, PROP_UNIQUE_ID,
      g_param_spec_uint ("unique-id",
          "Unique ID",
          "Unique ID for the element. Can be used to identify output of the"
          " element", 0, G_MAXUINT, DEFAULT_UNIQUE_ID, (GParamFlags)
          (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_PROCESSING_WIDTH,
      g_param_spec_int ("processing-width",
          "Processing Width",
          "Width of the input buffer to algorithm",
          1, G_MAXINT, DEFAULT_PROCESSING_WIDTH, (GParamFlags)
          (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_PROCESSING_HEIGHT,
      g_param_spec_int ("processing-height",
          "Processing Height",
          "Height of the input buffer to algorithm",
          1, G_MAXINT, DEFAULT_PROCESSING_HEIGHT, (GParamFlags)
          (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_PROCESS_FULL_FRAME,
      g_param_spec_boolean ("full-frame",
          "Full frame",
          "Enable to process full frame or disable to process objects detected"
          "by primary detector", DEFAULT_PROCESS_FULL_FRAME, (GParamFlags)
          (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_GPU_DEVICE_ID,
      g_param_spec_uint ("gpu-id",
          "Set GPU Device ID",
          "Set GPU Device ID", 0,
          G_MAXUINT, 0,
          GParamFlags
          (G_PARAM_READWRITE |
              G_PARAM_STATIC_STRINGS | GST_PARAM_MUTABLE_READY)));
  g_object_class_install_property (gobject_class, PROP_NODE_NAME,
    g_param_spec_string ("node-name", "Node Name",
              "Name of the rclcpp publisher node (default = gst_publisher)",
              "gst_publisher", static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property (gobject_class, PROP_TOPIC_NAME,
    g_param_spec_string ("topic-name", "Topic Name",
              "Name of rclcpp topic (default = image)",
              "image", static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  /* Set sink and src pad capabilities */
  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&gst_ds_rclcpp_publisher_src_template));
  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&gst_ds_rclcpp_publisher_sink_template));

  /* Set metadata describing the element */
  gst_element_class_set_details_simple (gstelement_class,
      "DsExample plugin",
      "DsExample Plugin",
      "Process a 3rdparty example algorithm on objects / full frame",
      "NVIDIA Corporation. Post on Deepstream for Tesla forum for any queries "
      "@ https://devtalk.nvidia.com/default/board/209/");
}

static void
gst_ds_rclcpp_publisher_init (GstDsRclcppPublisher * ds_rclcpp_publisher)
{
  GstBaseTransform *btrans = GST_BASE_TRANSFORM (ds_rclcpp_publisher);
  rclcpp::init(0, nullptr);
  ds_rclcpp_publisher->node = std::make_unique<GstDsRclcppPublisherNode>("ds_gst_publisher", "metadata");

  /* We will not be generating a new buffer. Just adding / updating
   * metadata. */
  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (btrans), TRUE);
  /* We do not want to change the input caps. Set to passthrough. transform_ip
   * is still called. */
  gst_base_transform_set_passthrough (GST_BASE_TRANSFORM (btrans), TRUE);

  /* Initialize all property variables to default values */
  ds_rclcpp_publisher->unique_id = DEFAULT_UNIQUE_ID;
  ds_rclcpp_publisher->processing_width = DEFAULT_PROCESSING_WIDTH;
  ds_rclcpp_publisher->processing_height = DEFAULT_PROCESSING_HEIGHT;
  ds_rclcpp_publisher->process_full_frame = DEFAULT_PROCESS_FULL_FRAME;
  ds_rclcpp_publisher->gpu_id = DEFAULT_GPU_ID;
  /* This quark is required to identify NvDsMeta when iterating through
   * the buffer metadatas */
  if (!_dsmeta_quark)
    _dsmeta_quark = g_quark_from_static_string (NVDS_META_STRING);
}

/* Function called when a property of the element is set. Standard boilerplate.
 */
static void
gst_ds_rclcpp_publisher_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  std::cout << "Setting property: " << prop_id << std::endl;
  GstDsRclcppPublisher *ds_rclcpp_publisher = GST_DS_RCLCPP_PUBLISHER (object);
  switch (prop_id) {
    case PROP_UNIQUE_ID:
      ds_rclcpp_publisher->unique_id = g_value_get_uint (value);
      break;
    case PROP_PROCESSING_WIDTH:
      ds_rclcpp_publisher->processing_width = g_value_get_int (value);
      break;
    case PROP_PROCESSING_HEIGHT:
      ds_rclcpp_publisher->processing_height = g_value_get_int (value);
      break;
    case PROP_PROCESS_FULL_FRAME:
      ds_rclcpp_publisher->process_full_frame = g_value_get_boolean (value);
      break;
    case PROP_GPU_DEVICE_ID:
      ds_rclcpp_publisher->gpu_id = g_value_get_uint (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* Function called when a property of the element is requested. Standard
 * boilerplate.
 */
static void
gst_ds_rclcpp_publisher_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstDsRclcppPublisher *ds_rclcpp_publisher = GST_DS_RCLCPP_PUBLISHER (object);
  std::cout << "Getting property: " << prop_id << std::endl;
  switch (prop_id) {
    case PROP_UNIQUE_ID:
      g_value_set_uint (value, ds_rclcpp_publisher->unique_id);
      break;
    case PROP_PROCESSING_WIDTH:
      g_value_set_int (value, ds_rclcpp_publisher->processing_width);
      break;
    case PROP_PROCESSING_HEIGHT:
      g_value_set_int (value, ds_rclcpp_publisher->processing_height);
      break;
    case PROP_PROCESS_FULL_FRAME:
      g_value_set_boolean (value, ds_rclcpp_publisher->process_full_frame);
      break;
    case PROP_GPU_DEVICE_ID:
      g_value_set_uint (value, ds_rclcpp_publisher->gpu_id);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/**
 * Initialize all resources and start the output thread
 */
static gboolean
gst_ds_rclcpp_publisher_start (GstBaseTransform * btrans)
{
  GstDsRclcppPublisher *ds_rclcpp_publisher = GST_DS_RCLCPP_PUBLISHER (btrans);
  NvBufSurfaceCreateParams create_params;
  // DsExampleInitParams init_params =
  //     { ds_rclcpp_publisher->processing_width, ds_rclcpp_publisher->processing_height,
  //   ds_rclcpp_publisher->process_full_frame
  // };

  GstQuery *queryparams = NULL;
  guint batch_size = 1;

  /* Algorithm specific initializations and resource allocation. */
  // ds_rclcpp_publisher->dsexamplelib_ctx = DsExampleCtxInit (&init_params);

  // GST_DEBUG_OBJECT (ds_rclcpp_publisher, "ctx lib %p \n", ds_rclcpp_publisher->dsexamplelib_ctx);

  CHECK_CUDA_STATUS (cudaSetDevice (ds_rclcpp_publisher->gpu_id),
      "Unable to set cuda device");

  ds_rclcpp_publisher->batch_size = 1;
  queryparams = gst_nvquery_batch_size_new ();
  if (gst_pad_peer_query (GST_BASE_TRANSFORM_SINK_PAD (btrans), queryparams)
      || gst_pad_peer_query (GST_BASE_TRANSFORM_SRC_PAD (btrans), queryparams)) {
    if (gst_nvquery_batch_size_parse (queryparams, &batch_size)) {
      ds_rclcpp_publisher->batch_size = batch_size;
    }
  }
  GST_DEBUG_OBJECT (ds_rclcpp_publisher, "Setting batch-size %d \n",
      ds_rclcpp_publisher->batch_size);
  gst_query_unref (queryparams);

  CHECK_CUDA_STATUS (cudaStreamCreate (&ds_rclcpp_publisher->cuda_stream),
      "Could not create cuda stream");

  if (ds_rclcpp_publisher->inter_buf)
    NvBufSurfaceDestroy (ds_rclcpp_publisher->inter_buf);
  ds_rclcpp_publisher->inter_buf = NULL;

  /* An intermediate buffer for NV12/RGBA to BGR conversion  will be
   * required. Can be skipped if custom algorithm can work directly on NV12/RGBA. */
  create_params.gpuId  = ds_rclcpp_publisher->gpu_id;
  create_params.width  = ds_rclcpp_publisher->processing_width;
  create_params.height = ds_rclcpp_publisher->processing_height;
  create_params.size = 0;
  create_params.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
  create_params.layout = NVBUF_LAYOUT_PITCH;
#ifdef __aarch64__
  create_params.memType = NVBUF_MEM_DEFAULT;
#else
  create_params.memType = NVBUF_MEM_CUDA_UNIFIED;
#endif

  if (NvBufSurfaceCreate (&ds_rclcpp_publisher->inter_buf, 1,
          &create_params) != 0) {
    GST_ERROR ("Error: Could not allocate internal buffer for ds_rclcpp_publisher");
    goto error;
  }

  // Create host memory for storing converted/scaled interleaved RGB data
  CHECK_CUDA_STATUS (cudaMallocHost (&ds_rclcpp_publisher->host_rgb_buf,
          ds_rclcpp_publisher->processing_width * ds_rclcpp_publisher->processing_height *
          RGB_BYTES_PER_PIXEL), "Could not allocate cuda host buffer");

  GST_DEBUG_OBJECT (ds_rclcpp_publisher, "allocated cuda buffer %p \n",
      ds_rclcpp_publisher->host_rgb_buf);

  // CV Mat containing interleaved RGB data. This call does not allocate memory.
  // It uses host_rgb_buf as data.
  // ds_rclcpp_publisher->cvmat =
  //     new cv::Mat (ds_rclcpp_publisher->processing_height, ds_rclcpp_publisher->processing_width,
  //     CV_8UC3, ds_rclcpp_publisher->host_rgb_buf,
  //     ds_rclcpp_publisher->processing_width * RGB_BYTES_PER_PIXEL);

  // if (!ds_rclcpp_publisher->cvmat)
  //   goto error;

  GST_DEBUG_OBJECT (ds_rclcpp_publisher, "created CV Mat\n");

  return TRUE;
error:
  if (ds_rclcpp_publisher->host_rgb_buf) {
    cudaFreeHost (ds_rclcpp_publisher->host_rgb_buf);
    ds_rclcpp_publisher->host_rgb_buf = NULL;
  }

  if (ds_rclcpp_publisher->cuda_stream) {
    cudaStreamDestroy (ds_rclcpp_publisher->cuda_stream);
    ds_rclcpp_publisher->cuda_stream = NULL;
  }
  // if (ds_rclcpp_publisher->dsexamplelib_ctx)
  //   DsExampleCtxDeinit (ds_rclcpp_publisher->dsexamplelib_ctx);
  return FALSE;
}

/**
 * Stop the output thread and free up all the resources
 */
static gboolean
gst_ds_rclcpp_publisher_stop (GstBaseTransform * btrans)
{
  GstDsRclcppPublisher *ds_rclcpp_publisher = GST_DS_RCLCPP_PUBLISHER (btrans);

  if (ds_rclcpp_publisher->inter_buf)
    NvBufSurfaceDestroy(ds_rclcpp_publisher->inter_buf);
  ds_rclcpp_publisher->inter_buf = NULL;

  if (ds_rclcpp_publisher->cuda_stream)
    cudaStreamDestroy (ds_rclcpp_publisher->cuda_stream);
  ds_rclcpp_publisher->cuda_stream = NULL;

  // delete ds_rclcpp_publisher->cvmat;
  // ds_rclcpp_publisher->cvmat = NULL;

  if (ds_rclcpp_publisher->host_rgb_buf) {
    cudaFreeHost (ds_rclcpp_publisher->host_rgb_buf);
    ds_rclcpp_publisher->host_rgb_buf = NULL;
  }



  GST_DEBUG_OBJECT (ds_rclcpp_publisher, "deleted CV Mat \n");

  // Deinit the algorithm library
  // DsExampleCtxDeinit (ds_rclcpp_publisher->dsexamplelib_ctx);
  // ds_rclcpp_publisher->dsexamplelib_ctx = NULL;

  GST_DEBUG_OBJECT (ds_rclcpp_publisher, "ctx lib released \n");

  return TRUE;
}

/**
 * Called when source / sink pad capabilities have been negotiated.
 */
static gboolean
gst_ds_rclcpp_publisher_set_caps (GstBaseTransform * btrans, GstCaps * incaps,
    GstCaps * outcaps)
{
  std::cout << "Setting caps" << std::endl;
  GstDsRclcppPublisher *ds_rclcpp_publisher = GST_DS_RCLCPP_PUBLISHER (btrans);
  /* Save the input video information, since this will be required later. */
  gst_video_info_from_caps (&ds_rclcpp_publisher->video_info, incaps);

  CHECK_CUDA_STATUS (cudaSetDevice (ds_rclcpp_publisher->gpu_id),
      "Unable to set cuda device");



  return TRUE;

error:
  return FALSE;
}

/**
 * Scale the entire frame to the processing resolution maintaining aspect ratio.
 * Or crop and scale objects to the processing resolution maintaining the aspect
 * ratio. Remove the padding required by hardware and convert from RGBA to RGB
 * using openCV. These steps can be skipped if the algorithm can work with
 * padded data and/or can work with RGBA.
 */
static GstFlowReturn
get_converted_mat (GstDsRclcppPublisher * ds_rclcpp_publisher, NvBufSurface *input_buf, gint idx,
    NvOSD_RectParams * crop_rect_params, gdouble & ratio, gint input_width,
    gint input_height)
{
  NvBufSurfTransform_Error err;
  NvBufSurfTransformConfigParams transform_config_params;
  NvBufSurfTransformParams transform_params;
  NvBufSurfTransformRect src_rect;
  NvBufSurfTransformRect dst_rect;
  NvBufSurface ip_surf;
  // cv::Mat in_mat;
  ip_surf = *input_buf;

  ip_surf.numFilled = ip_surf.batchSize = 1;
  ip_surf.surfaceList = &(input_buf->surfaceList[idx]);

  gint src_left = GST_ROUND_UP_2(crop_rect_params->left);
  gint src_top = GST_ROUND_UP_2(crop_rect_params->top);
  gint src_width = GST_ROUND_DOWN_2(crop_rect_params->width);
  gint src_height = GST_ROUND_DOWN_2(crop_rect_params->height);

  // Maintain aspect ratio
  double hdest = ds_rclcpp_publisher->processing_width * src_height / (double) src_width;
  double wdest = ds_rclcpp_publisher->processing_height * src_width / (double) src_height;
  guint dest_width, dest_height;

  if (hdest <= ds_rclcpp_publisher->processing_height) {
    dest_width = ds_rclcpp_publisher->processing_width;
    dest_height = hdest;
  } else {
    dest_width = wdest;
    dest_height = ds_rclcpp_publisher->processing_height;
  }

  // Configure transform session parameters for the transformation
  transform_config_params.compute_mode = NvBufSurfTransformCompute_Default;
  transform_config_params.gpu_id = ds_rclcpp_publisher->gpu_id;
  transform_config_params.cuda_stream = ds_rclcpp_publisher->cuda_stream;

  // Set the transform session parameters for the conversions executed in this
  // thread.
  err = NvBufSurfTransformSetSessionParams (&transform_config_params);
  if (err != NvBufSurfTransformError_Success) {
    GST_ELEMENT_ERROR (ds_rclcpp_publisher, STREAM, FAILED,
        ("NvBufSurfTransformSetSessionParams failed with error %d", err), (NULL));
    goto error;
  }

  // Calculate scaling ratio while maintaining aspect ratio
  ratio = MIN (1.0 * dest_width/ src_width, 1.0 * dest_height / src_height);

  if ((crop_rect_params->width == 0) || (crop_rect_params->height == 0)) {
    GST_ELEMENT_ERROR (ds_rclcpp_publisher, STREAM, FAILED,
        ("%s:crop_rect_params dimensions are zero",__func__), (NULL));
    goto error;
  }

#ifdef __aarch64__
  if (ratio <= 1.0 / 16 || ratio >= 16.0) {
    // Currently cannot scale by ratio > 16 or < 1/16 for Jetson
    goto error;
  }
#endif
  // Set the transform ROIs for source and destination
  src_rect = {(guint)src_top, (guint)src_left, (guint)src_width, (guint)src_height};
  dst_rect = {0, 0, (guint)dest_width, (guint)dest_height};

  // Set the transform parameters
  transform_params.src_rect = &src_rect;
  transform_params.dst_rect = &dst_rect;
  transform_params.transform_flag =
    NVBUFSURF_TRANSFORM_FILTER | NVBUFSURF_TRANSFORM_CROP_SRC |
      NVBUFSURF_TRANSFORM_CROP_DST;
  transform_params.transform_filter = NvBufSurfTransformInter_Default;

  //Memset the memory
  NvBufSurfaceMemSet (ds_rclcpp_publisher->inter_buf, 0, 0, 0);

  GST_DEBUG_OBJECT (ds_rclcpp_publisher, "Scaling and converting input buffer\n");

  // Transformation scaling+format conversion if any.
  err = NvBufSurfTransform (&ip_surf, ds_rclcpp_publisher->inter_buf, &transform_params);
  if (err != NvBufSurfTransformError_Success) {
    GST_ELEMENT_ERROR (ds_rclcpp_publisher, STREAM, FAILED,
        ("NvBufSurfTransform failed with error %d while converting buffer", err),
        (NULL));
    goto error;
  }
  // Map the buffer so that it can be accessed by CPU
  if (NvBufSurfaceMap (ds_rclcpp_publisher->inter_buf, 0, 0, NVBUF_MAP_READ) != 0){
    goto error;
  }

  // Cache the mapped data for CPU access
  NvBufSurfaceSyncForCpu (ds_rclcpp_publisher->inter_buf, 0, 0);

  // Use openCV to remove padding and convert RGBA to BGR. Can be skipped if
  // algorithm can handle padded RGBA data.
  // in_mat =
  //     cv::Mat (ds_rclcpp_publisher->processing_height, ds_rclcpp_publisher->processing_width,
  //     CV_8UC4, ds_rclcpp_publisher->inter_buf->surfaceList[0].mappedAddr.addr[0],
  //     ds_rclcpp_publisher->inter_buf->surfaceList[0].pitch);

// #if (CV_MAJOR_VERSION >= 4)
//   cv::cvtColor (in_mat, *ds_rclcpp_publisher->cvmat, cv::COLOR_RGBA2BGR);
// #else
//   cv::cvtColor (in_mat, *ds_rclcpp_publisher->cvmat, CV_RGBA2BGR);
// #endif

  if (NvBufSurfaceUnMap (ds_rclcpp_publisher->inter_buf, 0, 0)){
    goto error;
  }

#ifdef __aarch64__
  // To use the converted buffer in CUDA, create an EGLImage and then use
  // CUDA-EGL interop APIs
  if (USE_EGLIMAGE) {
    if (NvBufSurfaceMapEglImage (ds_rclcpp_publisher->inter_buf, 0) !=0 ) {
      goto error;
    }

    // ds_rclcpp_publisher->inter_buf->surfaceList[0].mappedAddr.eglImage
    // Use interop APIs cuGraphicsEGLRegisterImage and
    // cuGraphicsResourceGetMappedEglFrame to access the buffer in CUDA

    // Destroy the EGLImage
    NvBufSurfaceUnMapEglImage (ds_rclcpp_publisher->inter_buf, 0);
  }
#endif

  /* We will first convert only the Region of Interest (the entire frame or the
   * object bounding box) to RGB and then scale the converted RGB frame to
   * processing resolution. */
  return GST_FLOW_OK;

error:
  return GST_FLOW_ERROR;
}

/**
 * Called when element recieves an input buffer from upstream element.
 */
static GstFlowReturn
gst_ds_rclcpp_publisher_transform_ip (GstBaseTransform * btrans, GstBuffer * inbuf)
{
  GstDsRclcppPublisher *ds_rclcpp_publisher = GST_DS_RCLCPP_PUBLISHER (btrans);
  GstMapInfo in_map_info;
  GstFlowReturn flow_ret = GST_FLOW_ERROR;
  gdouble scale_ratio = 1.0;

  NvBufSurface *surface = NULL;
  NvDsBatchMeta *batch_meta = NULL;
  NvDsFrameMeta *data = NULL;
  NvDsMetaList * l_frame = NULL;
  guint i = 0;

  ds_rclcpp_publisher->frame_num++;
  CHECK_CUDA_STATUS (cudaSetDevice (ds_rclcpp_publisher->gpu_id),
      "Unable to set cuda device");

  memset (&in_map_info, 0, sizeof (in_map_info));
  if (!gst_buffer_map (inbuf, &in_map_info, GST_MAP_READ)) {
    g_print ("Error: Failed to map gst buffer\n");
    goto error;
  }

  surface = (NvBufSurface *) in_map_info.data;
  GST_DEBUG_OBJECT (ds_rclcpp_publisher,
      "Processing Frame %" G_GUINT64_FORMAT " Surface %p\n",
      ds_rclcpp_publisher->frame_num, surface);

  if (CHECK_NVDS_MEMORY_AND_GPUID (ds_rclcpp_publisher, surface))
    goto error;

  batch_meta = gst_buffer_get_nvds_batch_meta (inbuf);
  if (batch_meta == nullptr) {
    GST_ELEMENT_ERROR (ds_rclcpp_publisher, STREAM, FAILED,
        ("NvDsBatchMeta not found for input buffer."), (NULL));
    return GST_FLOW_ERROR;
  }

  //if (ds_rclcpp_publisher->process_full_frame) {
    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
      l_frame = l_frame->next)
    {
      data = (NvDsFrameMeta *) (l_frame->data);
      GstDsRclcppPublisherNode::ProcessNewFrame(ds_rclcpp_publisher->node.get(), data);
    }
      // NvOSD_RectParams rect_params;
      //
      // // Scale the entire frame to processing resolution
      // rect_params.left = 0;
      // rect_params.top = 0;
      // rect_params.width = ds_rclcpp_publisher->video_info.width;
      // rect_params.height = ds_rclcpp_publisher->video_info.height;
      //
      // // Scale and convert the frame
      // if (get_converted_mat (ds_rclcpp_publisher, surface, i, &rect_params,
      //       scale_ratio, ds_rclcpp_publisher->video_info.width,
      //       ds_rclcpp_publisher->video_info.height) != GST_FLOW_OK) {
      //   goto error;
      // }
      //
      // // Process to get the output
      // // output =
      // //     DsExampleProcess (ds_rclcpp_publisher->dsexamplelib_ctx,
      // //     ds_rclcpp_publisher->cvmat->data);
      // // Attach the metadata for the full frame
      // attach_metadata_full_frame (ds_rclcpp_publisher, data, scale_ratio, i);
      // i++;
      // free (output);
    //}

  // } else {
  //   // Using object crops as input to the algorithm. The objects are detected by
  //   // the primary detector
  //   NvDsMetaList * l_obj = NULL;
  //   NvDsObjectMeta *obj_meta = NULL;
  //
  //   for (l_frame = batch_meta->data_list; l_frame != NULL;
  //     l_frame = l_frame->next)
  //   {
  //     data = (NvDsFrameMeta *) (l_frame->data);
  //     for (l_obj = data->obj_meta_list; l_obj != NULL;
  //         l_obj = l_obj->next)
  //     {
  //       obj_meta = (NvDsObjectMeta *) (l_obj->data);
  //
  //
  //       /* Should not process on objects smaller than MIN_INPUT_OBJECT_WIDTH x MIN_INPUT_OBJECT_HEIGHT
  //        * since it will cause hardware scaling issues. */
  //       if (obj_meta->rect_params.width < MIN_INPUT_OBJECT_WIDTH ||
  //           obj_meta->rect_params.height < MIN_INPUT_OBJECT_HEIGHT)
  //         continue;
  //
  //       // Crop and scale the object
  //       if (get_converted_mat (ds_rclcpp_publisher,
  //             surface, data->batch_id, &obj_meta->rect_params,
  //             scale_ratio, ds_rclcpp_publisher->video_info.width,
  //             ds_rclcpp_publisher->video_info.height) != GST_FLOW_OK) {
  //         // Error in conversion, skip processing on object. */
  //         continue;
  //       }
  //
  //       // Process the object crop to obtain label
  //       // output = DsExampleProcess (ds_rclcpp_publisher->dsexamplelib_ctx,
  //       //     ds_rclcpp_publisher->cvmat->data);
  //
  //       // Attach labels for the object
  //       attach_metadata_object (ds_rclcpp_publisher, obj_meta);
  //
  //       // free (output);
  //     }
  //   }
  // }

  flow_ret = GST_FLOW_OK;

error:
  gst_buffer_unmap (inbuf, &in_map_info);
  return flow_ret;
}

/**
 * Attach metadata for the full frame. We will be adding a new metadata.
 */
static void
attach_metadata_full_frame (GstDsRclcppPublisher * ds_rclcpp_publisher, NvDsFrameMeta *data,
    gdouble scale_ratio, guint batch_id)
{
  NvDsBatchMeta *batch_meta = data->base_meta.batch_meta;
  NvDsObjectMeta *object_meta = NULL;
  static gchar font_name[] = "Serif";
  // GST_DEBUG_OBJECT (ds_rclcpp_publisher, "Attaching metadata %d\n", output->numObjects);

  // for (gint i = 0; i < output->numObjects; i++) {
  //   DsExampleObject *obj = &output->object[i];
  //   object_meta = nvds_acquire_obj_meta_from_pool(batch_meta);
  //   NvOSD_RectParams & rect_params = object_meta->rect_params;
  //   NvOSD_TextParams & text_params = object_meta->text_params;
  //
  //   // Assign bounding box coordinates
  //   rect_params.left = obj->left;
  //   rect_params.top = obj->top;
  //   rect_params.width = obj->width;
  //   rect_params.height = obj->height;
  //
  //   // Semi-transparent yellow background
  //   rect_params.has_bg_color = 0;
  //   rect_params.bg_color = (NvOSD_ColorParams) {
  //   1, 1, 0, 0.4};
  //   // Red border of width 6
  //   rect_params.border_width = 3;
  //   rect_params.border_color = (NvOSD_ColorParams) {
  //   1, 0, 0, 1};
  //
  //   // Scale the bounding boxes proportionally based on how the object/frame was
  //   // scaled during input
  //   rect_params.left /= scale_ratio;
  //   rect_params.top /= scale_ratio;
  //   rect_params.width /= scale_ratio;
  //   rect_params.height /= scale_ratio;
  //   GST_DEBUG_OBJECT (ds_rclcpp_publisher, "Attaching rect%d of batch%u"
  //       "  left->%u top->%u width->%u"
  //       " height->%u label->%s\n", i, batch_id, rect_params.left,
  //       rect_params.top, rect_params.width, rect_params.height, obj->label);
  //
  //   object_meta->object_id = UNTRACKED_OBJECT_ID;
  //   g_strlcpy (object_meta->obj_label, obj->label, MAX_LABEL_SIZE);
  //   // display_text required heap allocated memory
  //   text_params.display_text = g_strdup (obj->label);
  //   // Display text above the left top corner of the object
  //   text_params.x_offset = rect_params.left;
  //   text_params.y_offset = rect_params.top - 10;
  //   // Set black background for the text
  //   text_params.set_bg_clr = 1;
  //   text_params.text_bg_clr = (NvOSD_ColorParams) {
  //   0, 0, 0, 1};
  //   // Font face, size and color
  //   text_params.font_params.font_name = font_name;
  //   text_params.font_params.font_size = 11;
  //   text_params.font_params.font_color = (NvOSD_ColorParams) {
  //   1, 1, 1, 1};
  //
  //   nvds_add_obj_meta_to_frame(data, object_meta, NULL);
  // }
}

/**
 * Only update string label in an existing object metadata. No bounding boxes.
 * We assume only one label per object is generated
 */
static void
attach_metadata_object (GstDsRclcppPublisher * ds_rclcpp_publisher, NvDsObjectMeta * obj_meta)
{
  // if (output->numObjects == 0)
  //   return;
  NvDsBatchMeta *batch_meta = obj_meta->base_meta.batch_meta;

  NvDsClassifierMeta *classifier_meta =
    nvds_acquire_classifier_meta_from_pool (batch_meta);

  classifier_meta->unique_component_id = ds_rclcpp_publisher->unique_id;

  NvDsLabelInfo *label_info =
    nvds_acquire_label_info_meta_from_pool (batch_meta);
  // g_strlcpy (label_info->result_label, output->object[0].label, MAX_LABEL_SIZE);
  nvds_add_label_info_meta_to_classifier(classifier_meta, label_info);
  nvds_add_classifier_meta_to_object (obj_meta, classifier_meta);

  nvds_acquire_meta_lock (batch_meta);
  NvOSD_TextParams & text_params = obj_meta->text_params;
  NvOSD_RectParams & rect_params = obj_meta->rect_params;

  /* Below code to display the result */
  // Set black background for the text
  // display_text required heap allocated memory
  if (text_params.display_text) {
    // gchar *conc_string = g_strconcat (text_params.display_text, " ",
    //     output->object[0].label, NULL);
    // g_free (text_params.display_text);
    // text_params.display_text = conc_string;
  } else {
    // Display text above the left top corner of the object
    text_params.x_offset = rect_params.left;
    text_params.y_offset = rect_params.top - 10;
    // text_params.display_text = g_strdup (output->object[0].label);
    // Font face, size and color
    text_params.font_params.font_name = (char *)"Serif";
    text_params.font_params.font_size = 11;
    text_params.font_params.font_color = (NvOSD_ColorParams) {
    1, 1, 1, 1};
    // Set black background for the text
    text_params.set_bg_clr = 1;
    text_params.text_bg_clr = (NvOSD_ColorParams) {
    0, 0, 0, 1};
  }
  nvds_release_meta_lock (batch_meta);
}

/**
 * Boiler plate for registering a plugin and an element.
 */
static gboolean
ds_rclcpp_publisher_plugin_init (GstPlugin * plugin)
{
  GST_DEBUG_CATEGORY_INIT (gst_ds_rclcpp_publisher_debug, "ds_rclcpp_publisher", 0,
      "ds_rclcpp_publisher plugin");

  return gst_element_register (plugin, "ds_rclcpp_publisher", GST_RANK_PRIMARY,
      GST_TYPE_DS_RCLCPP_PUBLISHER);
}

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    ds_rclcpp_publisher,
    DESCRIPTION, ds_rclcpp_publisher_plugin_init, "4.0", LICENSE, BINARY_PACKAGE, URL)

GstDsRclcppPublisherNode::GstDsRclcppPublisherNode(const std::string& name, const std::string& topic_name)
: rclcpp::Node(name), publisher_(nullptr) {
    publisher_ = create_publisher<ros2_deepstream_msgs::msg::NvDsMetaData>(topic_name, 1);
}

GstDsRclcppPublisherNode::~GstDsRclcppPublisherNode() {}

void GstDsRclcppPublisherNode::Publish(ros2_deepstream_msgs::msg::NvDsMetaData::UniquePtr msg) {
  publisher_->publish(std::move(msg));
}

void GstDsRclcppPublisherNode::ProcessNewFrame(GstDsRclcppPublisherNode* node, const NvDsFrameMeta* data) {
  return;
  if (data == nullptr || data->obj_meta_list == nullptr) {
    return;
  }
  ros2_deepstream_msgs::msg::NvDsMetaData::UniquePtr msg = std::make_unique<ros2_deepstream_msgs::msg::NvDsMetaData>();

  // NvDsBatchMeta *batch_meta = data->base_meta.batch_meta;
  // NvDsObjectMeta* object_meta = nvds_acquire_obj_meta_from_pool(batch_meta);
  for (NvDsMetaList * l_obj = data->obj_meta_list; l_obj != NULL; l_obj = l_obj->next)
  {
    if (l_obj->data == nullptr) {
      std::cout << "Bad rect data" << std::endl;
      continue;
    }
    NvDsObjectMeta *obj_meta = static_cast<NvDsObjectMeta*>(l_obj->data);
    if (obj_meta == nullptr)  {
      std::cout << "Bad obj meta data" << std::endl;
      continue;
    }
    const std::string label = (obj_meta->obj_label == nullptr) ? "" : obj_meta->obj_label;
    sensor_msgs::msg::RegionOfInterest region;

    region.x_offset = obj_meta->rect_params.left;
    region.y_offset = obj_meta->rect_params.top;
    region.width = obj_meta->rect_params.width;
    region.height = obj_meta->rect_params.height;
    msg->regions.push_back(region);
    msg->labels.push_back(label);
  }

  node->Publish(std::move(msg));
}
