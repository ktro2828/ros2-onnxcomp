from __future__ import annotations

import argparse
import os.path as osp

import onnx
import yaml


def _select_layer(model) -> onnx.NodeProto:
    print(">>> Show layers...")
    source_layers = {}
    for idx, layer in enumerate(model.graph.node):
        print(
            f"Layer {idx}:"
            f"  Name={layer.name}, OpType={layer.op_type}, In={layer.input}, Out={layer.output}"
        )
        source_layers[layer.name] = layer

    while True:
        target_name = input("Select layer name: ")
        print(target_name)
        if target_name in source_layers:
            break
        elif not target_name:
            print("Invalid input, please try again")
        else:
            print(f"{target_name} is not a valid layer name")

    return source_layers[target_name]


def _update_output(model, target_layer) -> onnx.ModelProto:
    target_output_name = target_layer.output[0]

    # fill the value_info with shape information
    model = onnx.shape_inference.infer_shapes(model)

    # check if the target output is already a graph output
    existing_output_names = [o.name for o in model.graph.output]
    if target_output_name in existing_output_names:
        print(f"{target_output_name} is already a graph output. Skip adding.")
        return model

    # search for the target output in value_info
    target_value_info = None
    for vi in model.graph.value_info:
        if vi.name == target_output_name:
            target_value_info = vi
            break

    # if there is nothing in value_info, create a new ValueInfo with unknown shape
    if target_value_info is None:
        target_value_info = onnx.helper.make_tensor_value_info(
            target_output_name,
            onnx.TensorProto.FLOAT,
            None,  # unknown shape
        )

    model.graph.output.append(target_value_info)

    return model


def main() -> None:
    parser = argparse.ArgumentParser(description="Select a layer from an ONNX model")
    parser.add_argument("model_path", help="Path to the ONNX model")
    args = parser.parse_args()

    model = onnx.load(args.model_path)

    target_layer = _select_layer(model)
    print(f">>> Selected layer: {target_layer.name}")

    model = _update_output(model, target_layer)

    source_filename, ext = osp.splitext(osp.basename(args.model_path))
    target_layer_filename = target_layer.name.replace("/", "_")
    output_filename = f"{source_filename}_{target_layer_filename}"
    output_path = f"onnxcomp/data/{output_filename}{ext}"
    onnx.save(model, output_path)
    print(f">>> Model saved to {output_path}")

    config = dict(
        original_path=args.model_path,
        modified_path=output_path,
        layer_name=target_layer.name,
    )

    ros_param = {"/**": {"ros__parameters": config}}

    with open(f"onnxcomp/config/{output_filename}.param.yaml", "w") as f:
        yaml.dump(ros_param, f)


if __name__ == "__main__":
    main()
