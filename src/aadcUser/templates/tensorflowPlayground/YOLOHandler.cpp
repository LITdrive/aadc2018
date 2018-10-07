//
// Created by aadc on 21.09.18.
//

#include "YOLOHandler.h"

Status YOLOHandler::load_graph() {
    string graph = "data/frozen-yolo-voc.pb";
    string root_dir = "/home/aadc/share/adtf/src/aadcUser/templates/tensorflowPlayground";
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

// Convert Mat image into tensor of shape (1, height, width, d) where last three dims are equal to the original dims.
Tensor readTensorFromMat(const Mat &mat) {
    int height = mat.rows;
    int width = mat.cols;
    int depth = mat.channels();
    Tensor inputTensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1, height, width, depth}));
    auto inputTensorMapped = inputTensor.tensor<float, 4>();

    cv::Mat frame;
    mat.convertTo(frame, CV_8UC3);
    const tensorflow::uint8* source_data = (tensorflow::uint8*)frame.data;
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

    //resize(camera_image, image, Size(height, width));
    int x = 380;
    int x_plus_offset = 448;
    cv::Rect myROI(x, x, x_plus_offset, x_plus_offset);
    image = camera_image(myROI);



    inputTensor = readTensorFromMat(image);

    Status run_status = session->Run({{"input", inputTensor}},
                                        {"output"}, {}, &outputs);
    if (!run_status.ok()) {
        LOG(ERROR) << "Running model failed: " << run_status;
        return Tensor();
    }

    return outputs[0];
}

int main(int argc, char* argv[]) {
    Mat image = imread("/home/aadc/Downloads/pic.jpg");
    YOLOHandler yolo_handler;
    Status load_graph_status = yolo_handler.load_graph();
    if (!load_graph_status.ok()) {
        std::cout << load_graph_status.ToString() << "\n";
    }

    Tensor output = yolo_handler.forward_path(image);

    std::cout << output.flat<float>();
}