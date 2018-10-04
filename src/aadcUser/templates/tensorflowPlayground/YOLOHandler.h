//
// Created by aadc on 21.09.18.
//

#ifndef AADC_USER_YOLOHANDLER_H
#define AADC_USER_YOLOHANDLER_H


#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/strings/str_util.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/util/command_line_flags.h"

#include <opencv2/opencv.hpp>

using namespace tensorflow;
using namespace cv;

class YOLOHandler {
    std::unique_ptr<Session> session;
    GraphDef graph_def;

public:
    Status load_graph();
    Tensor forward_path(Mat image);
};


#endif //AADC_USER_YOLOHANDLER_H
