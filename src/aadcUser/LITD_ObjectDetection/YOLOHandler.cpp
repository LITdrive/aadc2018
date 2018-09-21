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

Tensor YOLOHandler::forward_path(Mat image) {
    int height = 488;
    int width = 488;
    std::vector<Tensor> outputs;

    resize(image, image, Size(height, width));

    int depth = image.channels();

    // creating a Tensor for storing the data
    tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,height,width,depth}));
    auto input_tensor_mapped = input_tensor.tensor<float, 4>();
    const float * source_data = (float*) image.data;

    // copying the data into the corresponding tensor
//    for (int y = 0; y < height; ++y) {
//        const float* source_row = source_data + (y * width * depth);
//        for (int x = 0; x < width; ++x) {
//            const float* source_pixel = source_row + (x * depth);
//            for (int c = 0; c < depth; ++c) {
//                const float* source_value = source_pixel + c;
//                input_tensor_mapped(0, y, x, c) = *source_value;
//            }
//        }
//    }

    Status run_status = session->Run({{"input", input_tensor}},
                                        {"output"}, {}, &outputs);
    if (!run_status.ok()) {
        LOG(ERROR) << "Running model failed: " << run_status;
        return Tensor();
    }

    return outputs[0];
}
