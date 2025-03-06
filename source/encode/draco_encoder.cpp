#pragma once
#include "draco/compression/config/compression_shared.h"
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/file_utils.h"
#include "draco/io/mesh_io.h"
#include "draco/io/point_cloud_io.h"
#include "draco_encoder.h"

void PrintOptions(const draco::PointCloud& pc, const Options& options) {
    printf("Encoder options:\n");
    printf("  Compression level = %d\n", options.compression_level);
    if (options.pos_quantization_bits == 0) {
        printf("  Positions: No quantization\n");
    }
    else {
        printf("  Positions: Quantization = %d bits\n",
            options.pos_quantization_bits);
    }

    if (pc.GetNamedAttributeId(draco::GeometryAttribute::TEX_COORD) >= 0) {
        if (options.tex_coords_quantization_bits == 0) {
            printf("  Texture coordinates: No quantization\n");
        }
        else {
            printf("  Texture coordinates: Quantization = %d bits\n",
                options.tex_coords_quantization_bits);
        }
    }
    else if (options.tex_coords_deleted) {
        printf("  Texture coordinates: Skipped\n");
    }

    if (pc.GetNamedAttributeId(draco::GeometryAttribute::NORMAL) >= 0) {
        if (options.normals_quantization_bits == 0) {
            printf("  Normals: No quantization\n");
        }
        else {
            printf("  Normals: Quantization = %d bits\n",
                options.normals_quantization_bits);
        }
    }
    else if (options.normals_deleted) {
        printf("  Normals: Skipped\n");
    }

    if (pc.GetNamedAttributeId(draco::GeometryAttribute::GENERIC) >= 0) {
        if (options.generic_quantization_bits == 0) {
            printf("  Generic: No quantization\n");
        }
        else {
            printf("  Generic: Quantization = %d bits\n",
                options.generic_quantization_bits);
        }
    }
    else if (options.generic_deleted) {
        printf("  Generic: Skipped\n");
    }
    printf("\n");
}

int EncodePointCloudToFile(const draco::PointCloud& pc, const std::string& file,
    draco::ExpertEncoder* encoder) {
    draco::CycleTimer timer;
    // Encode the geometry.
    draco::EncoderBuffer buffer;
    timer.Start();
    const draco::Status status = encoder->EncodeToBuffer(&buffer);
    if (!status.ok()) {
        printf("Failed to encode the point cloud.\n");
        printf("%s\n", status.error_msg());
        return -1;
    }
    timer.Stop();
    // Save the encoded geometry into a file.
    if (!draco::WriteBufferToFile(buffer.data(), buffer.size(), file)) {
        printf("Failed to write the output file.\n");
        return -1;
    }
    printf("Encoded point cloud saved to %s (%" PRId64 " ms to encode).\n",
        file.c_str(), timer.GetInMs());
    printf("\nEncoded size = %zu bytes\n\n", buffer.size());
    return 0;
}

int EncodeMeshToFile(const draco::Mesh& mesh, const std::string& file,
    draco::ExpertEncoder* encoder) {
    draco::CycleTimer timer;
    // Encode the geometry.
    draco::EncoderBuffer buffer;
    timer.Start();
    const draco::Status status = encoder->EncodeToBuffer(&buffer);
    if (!status.ok()) {
        printf("Failed to encode the mesh.\n");
        printf("%s\n", status.error_msg());
        return -1;
    }
    timer.Stop();
    // Save the encoded geometry into a file.
    if (!draco::WriteBufferToFile(buffer.data(), buffer.size(), file)) {
        printf("Failed to create the output file.\n");
        return -1;
    }
    printf("Encoded mesh saved to %s (%" PRId64 " ms to encode).\n", file.c_str(),
        timer.GetInMs());
    printf("\nEncoded size = %zu bytes\n\n", buffer.size());
    return 0;
}
int encoder_draco(Options &options) {
    std::unique_ptr<draco::PointCloud> pc;
    draco::Mesh* mesh = nullptr;
    if (!options.is_point_cloud) {
        draco::Options load_options;
        load_options.SetBool("use_metadata", options.use_metadata);
        load_options.SetBool("preserve_polygons", options.preserve_polygons);
        auto maybe_mesh = draco::ReadMeshFromFile(options.input, load_options);
        if (!maybe_mesh.ok()) {
            printf("Failed loading the input mesh: %s.\n",
                maybe_mesh.status().error_msg());
            return -1;
        }
        mesh = maybe_mesh.value().get();
        pc = std::move(maybe_mesh).value();
    }
    else {
        auto maybe_pc = draco::ReadPointCloudFromFile(options.input);
        if (!maybe_pc.ok()) {
            printf("Failed loading the input point cloud: %s.\n",
                maybe_pc.status().error_msg());
            return -1;
        }
        pc = std::move(maybe_pc).value();
    }

    if (options.pos_quantization_bits < 0) {
        printf("Error: Position attribute cannot be skipped.\n");
        return -1;
    }

    // Delete attributes if needed. This needs to happen before we set any
    // quantization settings.
    if (options.tex_coords_quantization_bits < 0) {
        if (pc->NumNamedAttributes(draco::GeometryAttribute::TEX_COORD) > 0) {
            options.tex_coords_deleted = true;
        }
        while (pc->NumNamedAttributes(draco::GeometryAttribute::TEX_COORD) > 0) {
            pc->DeleteAttribute(
                pc->GetNamedAttributeId(draco::GeometryAttribute::TEX_COORD, 0));
        }
    }
    if (options.normals_quantization_bits < 0) {
        if (pc->NumNamedAttributes(draco::GeometryAttribute::NORMAL) > 0) {
            options.normals_deleted = true;
        }
        while (pc->NumNamedAttributes(draco::GeometryAttribute::NORMAL) > 0) {
            pc->DeleteAttribute(
                pc->GetNamedAttributeId(draco::GeometryAttribute::NORMAL, 0));
        }
    }
    if (options.generic_quantization_bits < 0) {
        if (pc->NumNamedAttributes(draco::GeometryAttribute::GENERIC) > 0) {
            options.generic_deleted = true;
        }
        while (pc->NumNamedAttributes(draco::GeometryAttribute::GENERIC) > 0) {
            pc->DeleteAttribute(
                pc->GetNamedAttributeId(draco::GeometryAttribute::GENERIC, 0));
        }
    }
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
    // If any attribute has been deleted, run deduplication of point indices again
    // as some points can be possibly combined.
    if (options.tex_coords_deleted || options.normals_deleted ||
        options.generic_deleted) {
        pc->DeduplicatePointIds();
    }
#endif

    // Convert compression level to speed (that 0 = slowest, 10 = fastest).
    const int speed = 10 - options.compression_level;

    draco::Encoder encoder;

    // Setup encoder options.
    if (options.pos_quantization_bits > 0) {
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,
            options.pos_quantization_bits);
    }
    if (options.tex_coords_quantization_bits > 0) {
        encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,
            options.tex_coords_quantization_bits);
    }
    if (options.normals_quantization_bits > 0) {
        encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,
            options.normals_quantization_bits);
    }
    if (options.generic_quantization_bits > 0) {
        encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,
            options.generic_quantization_bits);
    }
    encoder.SetSpeedOptions(speed, speed);

    if (options.output.empty()) {
        // Create a default output file by attaching .drc to the input file name.
        options.output = options.input + ".drc";
    }

    PrintOptions(*pc, options);

    const bool input_is_mesh = mesh && mesh->num_faces() > 0;

    // Convert to ExpertEncoder that allows us to set per-attribute options.
    std::unique_ptr<draco::ExpertEncoder> expert_encoder;
    if (input_is_mesh) {
        expert_encoder.reset(new draco::ExpertEncoder(*mesh));
    }
    else {
        expert_encoder.reset(new draco::ExpertEncoder(*pc));
    }
    expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));

    // Check if there is an attribute that stores polygon edges. If so, we disable
    // the default prediction scheme for the attribute as it actually makes the
    // compression worse.
    const int poly_att_id =
        pc->GetAttributeIdByMetadataEntry("name", "added_edges");
    if (poly_att_id != -1) {
        expert_encoder->SetAttributePredictionScheme(
            poly_att_id, draco::PredictionSchemeMethod::PREDICTION_NONE);
    }

    int ret = -1;

    if (input_is_mesh) {
        ret = EncodeMeshToFile(*mesh, options.output, expert_encoder.get());
    }
    else {
        ret = EncodePointCloudToFile(*pc, options.output, expert_encoder.get());
    }

    if (ret != -1 && options.compression_level < 10) {
        printf(
            "For better compression, increase the compression level up to '-cl 10' "
            ".\n\n");
    }


}