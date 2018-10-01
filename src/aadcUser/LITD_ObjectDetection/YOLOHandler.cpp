//
// Created by aadc on 21.09.18.
//

#include "YOLOHandler.h"

Status YOLOHandler::load_graph() {
    string graph = "data/frozen_model.pb";
    string root_dir = "/home/aadc/AADC/src/aadcUser/LITD_ObjectDetection/";
    string graph_path = tensorflow::io::JoinPath(root_dir, graph);

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

Tensor YOLOHandler::forward_path(Mat camera_image) {
    int height = 448;
    int width = 448;
    Mat image;
    std::vector<Tensor> outputs;

    resize(camera_image, image, Size(height, width));

    int depth = image.channels();

    // creating a Tensor for storing the data
    tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,height,width,depth}));
    auto input_tensor_mapped = input_tensor.tensor<float, 4>();
    const float * source_data = (float*) image.data;

    // copying the data into the corresponding tensor
    float minR = 255;
    float maxR = 0;
    for (int y = 0; y < height; ++y) {
        const float* source_row = source_data + (y * width * depth);
        for (int x = 0; x < width; ++x) {
            const float* source_pixel = source_row + (x * depth);
            const float* source_B = source_pixel + 0;
            const float* source_G = source_pixel + 1;
            const float* source_R = source_pixel + 2;

            input_tensor_mapped(0, y, x, 0) = *source_R;
            input_tensor_mapped(0, y, x, 1) = *source_G;
            input_tensor_mapped(0, y, x, 2) = *source_B;

            if ( input_tensor_mapped(0, y, x, 0) < minR ) {
                minR = input_tensor_mapped(0, y, x, 0);
            }

            if ( input_tensor_mapped(0, y, x, 0) > maxR ) {
                maxR = input_tensor_mapped(0, y, x, 0);
            }
        }
    }

    LOG(INFO) << "min " << minR << " max " << maxR << std::endl;

    Status run_status = session->Run({{"input", input_tensor}},
                                        {"output"}, {}, &outputs);
    if (!run_status.ok()) {
        LOG(ERROR) << "Running model failed: " << run_status;
        return Tensor();
    }

    return outputs[0];
}
