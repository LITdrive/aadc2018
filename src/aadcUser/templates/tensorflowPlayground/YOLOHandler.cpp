//
// Created by aadc on 21.09.18.
//

#include "YOLOHandler.h"
#include <stdlib.h>

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
Tensor readTensorFromMat(const Mat &left, const Mat &center, const Mat &right) {
    int height = 416;
    int width = 416;
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
    int edge_length = 416;
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

int main(int argc, char* argv[]) {
    // run on CPU
    // setenv("CUDA_VISIBLE_DEVICES", "", 1);
    Mat image = imread("/home/aadc/Downloads/pic.jpg");
    YOLOHandler yolo_handler;
    Status load_graph_status = yolo_handler.load_graph();
    if (!load_graph_status.ok()) {
        std::cout << load_graph_status.ToString() << "\n";
    }

    Tensor output = yolo_handler.forward_path(image);
    auto output_flat = output.flat_outer_dims<float, 3>();

    int size_output_array = 24500;
    float left_output_array[size_output_array];
    float center_output_array[size_output_array];
    float right_output_array[size_output_array];

    for (int i = 0; i < size_output_array; i++) {
        left_output_array[i] = output_flat(i);
        center_output_array[i] = output_flat(i+24500);
        right_output_array[i] = output_flat(i+49000);
    }

    std::cout << "1: " << output_flat(1,2,3) << std::endl;
    std::cout << "1: " << left_output_array[24499] << std::endl;
    std::cout << "1: " << center_output_array[24499] << std::endl;
    std::cout << "1: " << right_output_array[24499] << std::endl;
    std::cout << "1: " << output.dims() << std::endl;
    std::cout << "2: "  << output.dim_size(0) << std::endl;
    std::cout << "3: "  << output.dim_size(1) << std::endl;
    std::cout << "4: "  << output.dim_size(2) << std::endl;
    std::cout << "5: "  << output.dim_size(3) << std::endl;
}