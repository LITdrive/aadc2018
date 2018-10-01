//
// Created by aadc on 21.09.18.
//

#include "YOLOHandler.h"

Status YOLOHandler::load_graph() {
    string graph = "data/frozen-yolo-tiny-aadc.pb";
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

Tensor readTensorFromMat(const Mat &mat) {
    int height = mat.rows;
    int width = mat.cols;
    int depth = mat.channels();
    Tensor inputTensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, height, width, depth}));
    auto inputTensorMapped = inputTensor.tensor<float, 4>();

    const tensorflow::uint8* source_data = (tensorflow::uint8*)mat.data;

    for (int y=0; y<height; y++){
        const tensorflow::uint8* source_row = source_data + (y*width*depth);
        for (int x=0; x<width; x++){
            const tensorflow::uint8* source_pixel = source_row + (x*depth);

            const tensorflow::uint8* source_value_blue = source_pixel;
            const tensorflow::uint8* source_value_green = source_pixel + 1;
            const tensorflow::uint8* source_value_red = source_pixel + 2;

            inputTensorMapped(0, y, x, 0) = (*source_value_red)/255.;
            inputTensorMapped(0, y, x, 1) = (*source_value_green)/255.;
            inputTensorMapped(0, y, x, 2) = (*source_value_blue)/255.;
        }
    }

    return inputTensor;
}

Tensor YOLOHandler::forward_path(Mat camera_image) {
    int height = 448;
    int width = 448;
    Mat image;
    std::vector<Tensor> outputs;
    Tensor inputTensor;

    resize(camera_image, image, Size(height, width));
    inputTensor = readTensorFromMat(image);

    Status run_status = session->Run({{"input", inputTensor}},
                                        {"output"}, {}, &outputs);
    if (!run_status.ok()) {
        LOG(ERROR) << "Running model failed: " << run_status;
        return Tensor();
    }

    return outputs[0];
}
