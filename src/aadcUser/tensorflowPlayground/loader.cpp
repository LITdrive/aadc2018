//
// Created by aadc on 17.09.18.
//

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

using namespace tensorflow;

static Status ReadEntireFile(tensorflow::Env* env, const string& filename,
                             Tensor* output) {
    tensorflow::uint64 file_size = 0;
    TF_RETURN_IF_ERROR(env->GetFileSize(filename, &file_size));

    string contents;
    contents.resize(file_size);

    std::unique_ptr<tensorflow::RandomAccessFile> file;
    TF_RETURN_IF_ERROR(env->NewRandomAccessFile(filename, &file));

    tensorflow::StringPiece data;
    TF_RETURN_IF_ERROR(file->Read(0, file_size, &data, &(contents)[0]));
    if (data.size() != file_size) {
        return tensorflow::errors::DataLoss("Truncated read of '", filename,
                                            "' expected ", file_size, " got ",
                                            data.size());
    }
    output->scalar<string>()() = data.ToString();
    return Status::OK();
}

// Given an image file name, read in the data, try to decode it as an image,
// resize it to the requested size, and then scale the values as desired.
Status ReadTensorFromImageFile(const string& file_name, const int input_height,
                               const int input_width, const float input_mean,
                               const float input_std,
                               std::vector<Tensor>* out_tensors) {
    auto root = tensorflow::Scope::NewRootScope();
    using namespace ::tensorflow::ops;  // NOLINT(build/namespaces)

    string input_name = "file_reader";
    string output_name = "normalized";

    // read file_name into a tensor named input
    Tensor input(tensorflow::DT_STRING, tensorflow::TensorShape());
    TF_RETURN_IF_ERROR(
            ReadEntireFile(tensorflow::Env::Default(), file_name, &input));

    // use a placeholder to read input data
    auto file_reader =
            Placeholder(root.WithOpName("input"), tensorflow::DataType::DT_STRING);

    std::vector<std::pair<string, tensorflow::Tensor>> inputs = {
            {"input", input},
    };

    // Now try to figure out what kind of file it is and decode it.
    const int wanted_channels = 3;
    tensorflow::Output image_reader;
    if (tensorflow::str_util::EndsWith(file_name, ".png")) {
        image_reader = DecodePng(root.WithOpName("png_reader"), file_reader,
                                 DecodePng::Channels(wanted_channels));
    } else if (tensorflow::str_util::EndsWith(file_name, ".gif")) {
        // gif decoder returns 4-D tensor, remove the first dim
        image_reader =
                Squeeze(root.WithOpName("squeeze_first_dim"),
                        DecodeGif(root.WithOpName("gif_reader"), file_reader));
    } else if (tensorflow::str_util::EndsWith(file_name, ".bmp")) {
        image_reader = DecodeBmp(root.WithOpName("bmp_reader"), file_reader);
    } else {
        // Assume if it's neither a PNG nor a GIF then it must be a JPEG.
        image_reader = DecodeJpeg(root.WithOpName("jpeg_reader"), file_reader,
                                  DecodeJpeg::Channels(wanted_channels));
    }
    // Now cast the image data to float so we can do normal math on it.
    auto float_caster =
            Cast(root.WithOpName("float_caster"), image_reader, tensorflow::DT_FLOAT);
    // The convention for image ops in TensorFlow is that all images are expected
    // to be in batches, so that they're four-dimensional arrays with indices of
    // [batch, height, width, channel]. Because we only have a single image, we
    // have to add a batch dimension of 1 to the start with ExpandDims().
    auto dims_expander = ExpandDims(root, float_caster, 0);
    // Bilinearly resize the image to fit the required dimensions.
    auto resized = ResizeBilinear(
            root, dims_expander,
            Const(root.WithOpName("size"), {input_height, input_width}));
    // Subtract the mean and divide by the scale.
    Div(root.WithOpName(output_name), Sub(root, resized, {input_mean}),
        {input_std});

    // This runs the GraphDef network definition that we've just constructed, and
    // returns the results in the output tensor.
    tensorflow::GraphDef graph;
    TF_RETURN_IF_ERROR(root.ToGraphDef(&graph));

    std::unique_ptr<tensorflow::Session> session(
            tensorflow::NewSession(tensorflow::SessionOptions()));
    TF_RETURN_IF_ERROR(session->Create(graph));
    TF_RETURN_IF_ERROR(session->Run({inputs}, {output_name}, {}, out_tensors));
    return Status::OK();
}

Status LoadGraph(const string& graph_file_name, std::unique_ptr<tensorflow::Session>* session) {
    tensorflow::GraphDef graph_def;
    Status load_graph_status =
            ReadBinaryProto(tensorflow::Env::Default(), graph_file_name, &graph_def);
    if (!load_graph_status.ok()) {
        return tensorflow::errors::NotFound("Failed to load compute graph at '",
                                            graph_file_name, "'");
    }
    session->reset(tensorflow::NewSession(tensorflow::SessionOptions()));
    Status session_create_status = (*session)->Create(graph_def);
    if (!session_create_status.ok()) {
        return session_create_status;
    }

//    Get input shape
    auto shape = graph_def.node().Get(0).attr().at("shape").shape();
    for (int i = 0; i < shape.dim_size(); i++) {
        std::cout << shape.dim(i).size()<<std::endl;
    }

    // get tensors names
    int node_count = graph_def.node_size();
    for (int i = 0; i < node_count; i++)
    {
        auto n = graph_def.node(i);
        std::cout <<"Names : "<< n.name() << std::endl;

    }

    return Status::OK();
}

Status ForwardPath(std::unique_ptr<tensorflow::Session>* session, Tensor input_tensor) {
    // Actually run the image through the model.
    std::vector<Tensor> outputs;
    Status run_status = (*session)->Run({{"input", input_tensor}},
                                     {"output"}, {}, &outputs);
    if (!run_status.ok()) {
        LOG(ERROR) << "Running model failed: " << run_status;
        return run_status;
    }

    int count = outputs.size();
    for (int i = 0; i < count; i++)
    {
        Tensor n = outputs.at(i);
        std::cout <<"DIMS : " << i << " "<< n.dims() << std::endl;

    }

    Tensor scores = outputs[0];

    tensorflow::TTypes<float>::Flat scores_flat = scores.flat<float>();

    std::cout << scores.dim_size(0) << std::endl;
    std::cout << scores.dim_size(1) << std::endl;
    std::vector<string> labels = {"person", "car"};

//    for (int pos = 0; pos < 2; ++pos) {
//        const int label_index = indices_flat(pos);
//        const float score = scores_flat(pos);
//        LOG(INFO) << labels[label_index] << " (" << label_index << "): " << score;
//    }

    return Status::OK();
}


int main(int argc, char* argv[]) {
    string graph = "data/frozen_model.pb";
    string root_dir = "/home/aadc/AADC/src/aadcUser/tensorflowPlayground/";
    string graph_path = tensorflow::io::JoinPath(root_dir, graph);
    int input_width = 448;
    int input_height = 448;
    std::unique_ptr<tensorflow::Session> session;

//     Initialize a tensorflow session
//    Session* session;
//    Status status = NewSession(SessionOptions(), &session);
//    if (!status.ok()) {
//        std::cout << status.ToString() << "\n";
//        return 1;
//    }




    // We need to call this to set up global state for TensorFlow.
//    tensorflow::port::InitMain(argv[0], &argc, &argv);
//    if (argc > 1) {
//        LOG(ERROR) << "Unknown argument " << argv[1] << "\n";
//        return -1;
//    }

  // Read in the protobuf graph we exported
  // (The path seems to be relative to the cwd. Keep this in mind
  // when using `bazel run` since the cwd isn't where you call
  // `bazel run` but from inside a temp folder.)
  Status load_graph_status = LoadGraph(graph_path, &session);
  if (!load_graph_status.ok()) {
    std::cout << load_graph_status.ToString() << "\n";
    return -1;
  }

    // Get the image from disk as a float array of numbers, resized and normalized
    // to the specifications the main graph expects.
    std::vector<Tensor> resized_tensors;
    string image_path = tensorflow::io::JoinPath(root_dir, "data/car_img.jpg");
    Status read_tensor_status =
            ReadTensorFromImageFile(image_path, input_width, input_width, 0, 255, &resized_tensors);
    if (!read_tensor_status.ok()) {
        std::cout << read_tensor_status.ToString() << "\n";
        return -1;
    }
    const Tensor& resized_tensor = resized_tensors[0];

    Status forward_path_status = ForwardPath(&session, resized_tensor);
    if (!forward_path_status.ok()) {
        std::cout << forward_path_status.ToString() << "\n";
        return -1;
    }

//  // Add the graph to the session
//  status = session->Create(graph_def);
//  if (!status.ok()) {
//    std::cout << status.ToString() << "\n";
//    return 1;
//  }

  // Setup inputs and outputs:

  // Our graph doesn't require any inputs, since it specifies default values,
  // but we'll change an input to demonstrate.
//  toTensor a(DT_FLOAT, TensorShape());
//  a.scalar<float>()() = 3.0;
//
//  Tensor b(DT_FLOAT, TensorShape());
//  b.scalar<float>()() = 2.0;
//
//  std::vector<std::pair<string, tensorflow::Tensor>> inputs = {
//    { "a", a },
//    { "b", b },
//  };
//
//  // The session will initialize the outputs
//  std::vector<tensorflow::Tensor> outputs;
//
//  // Run the session, evaluating our "c" operation from the graph
//  status = session->Run(inputs, {"c"}, {}, &outputs);
//  if (!status.ok()) {
//    std::cout << status.ToString() << "\n";
//    return 1;
//  }
//
//  // Grab the first output (we only evaluated one graph node: "c")
//  // and convert the node to a scalar representation.
//  auto output_c = outputs[0].scalar<float>();
//
//  // (There are similar methods for vectors and matrices here:
//  // https://github.com/tensorflow/tensorflow/blob/master/tensorflow/core/public/tensor.h)
//
//  // Print the results
//  std::cout << outputs[0].DebugString() << "\n"; // Tensor<type: float shape: [] values: 30>
//  std::cout << output_c() << "\n"; // 30
//
//  // Free any resources used by the session
//  session->Close();
//  return 0;
}

