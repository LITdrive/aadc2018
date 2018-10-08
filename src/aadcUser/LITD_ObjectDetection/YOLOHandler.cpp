/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hart#$  $Date:: 2017-05-19 08:12:10#$ $Rev:: 63515   $
**********************************************************************/

#include "stdafx.h"
#include "YOLOHandler.h"

Status YOLOHandler::load_graph(string graph_path) {
    LOG_INFO("Loading tensorflow model from '%s'", graph_path.c_str());
    Status load_graph_status =
            ReadBinaryProto(tensorflow::Env::Default(), graph_path, &graph_def);
    if (!load_graph_status.ok()) {
        return tensorflow::errors::NotFound("Failed to load compute graph at '",
                                            graph_path, "'");
    }

    session.reset(tensorflow::NewSession(tensorflow::SessionOptions()));
    Status session_create_status = session->Create(graph_def);
    if (!session_create_status.ok()) {
        return session_create_status;
    }

    return Status::OK();
}

Tensor readTensorFromMat(const Mat &left, const Mat &center, const Mat &right) {
    int height = 448;
    int width = 448;
    int depth = 3;
    int batch = 3;
    Tensor inputTensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({batch, height, width, depth}));
    auto inputTensorMapped = inputTensor.tensor<float, 4>();

    const tensorflow::uint8 *left_source_data = (tensorflow::uint8 *) left.data;
    const tensorflow::uint8 *center_source_data = (tensorflow::uint8 *) center.data;
    const tensorflow::uint8 *right_source_data = (tensorflow::uint8 *) right.data;

    const tensorflow::uint8 *all_images[3] = {left_source_data, center_source_data, right_source_data};

    for (int b=0; b<batch; b++){
        const tensorflow::uint8 *source_data = all_images[b];
        for (int y = 0; y < height; y++) {
            const tensorflow::uint8 *source_row = source_data + (y * width * depth);
            for (int x = 0; x < width; x++) {
                const tensorflow::uint8 *source_pixel = source_row + (x * depth);

                const tensorflow::uint8 *source_value_blue = source_pixel;
                const tensorflow::uint8 *source_value_green = source_pixel + 1;
                const tensorflow::uint8 *source_value_red = source_pixel + 2;

                inputTensorMapped(b, y, x, 0) = (*source_value_red) / 255.;
                inputTensorMapped(b, y, x, 1) = (*source_value_green) / 255.;
                inputTensorMapped(b, y, x, 2) = (*source_value_blue) / 255.;
            }
        }
    }

    return inputTensor;
}

Tensor YOLOHandler::forward_path(Mat camera_image) {
    Mat left;
    Mat center;
    Mat right;
    std::vector<Tensor> outputs;
    Tensor inputTensor;

    int x = 50;
    int y = 380;
    int edge_length = 448;
    cv::Rect ROI_left(x, y, edge_length, edge_length);
    left = camera_image(ROI_left);
    x = 416;
    cv::Rect ROI_center(x, y, edge_length, edge_length);
    center = camera_image(ROI_center);
    x = 742;
    cv::Rect ROI_right(x, y, edge_length, edge_length);
    right = camera_image(ROI_right);

    inputTensor = readTensorFromMat(left, right, center);

    Status run_status = session->Run({{"input", inputTensor}},
                                        {"output"}, {}, &outputs);
    if (!run_status.ok()) {
        LOG(ERROR) << "Running model failed: " << run_status;
        return Tensor();
    }

    return outputs[0];
}
