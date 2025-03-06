# ****************************************************************** #
#                          AVS Mesh Compression                      #
# ****************************************************************** #

This is the description document of avs mesh encoder.


# 1.build 
    ./build.sh

# 2.folder
    |---dependencies
        |---cmake
        |---draco
        |---hpm-HPM-15.0
        |---patches
        |---stb
    |---source
        |---encode
        |---decode

# 3.dependencies

  # 3.1 draco
  git clone https://github.com/google/draco.git draco

  # 3.2 hpm
  Download from avs ftp remote library.

  # 3.3 stb
  git clone https://github.com/nothings/stb.git stb

# 4.encode/decode config

  Those marked * are required.

  # encode
  
  Main options:
  -h | -?                           show help.
  -iMesh <input mesh>              *input mesh file name.
  -iMap <input map>                 input texture map file name.
  -oMesh <output mesh>              output mesh bitstream.
  -oMap <output map>                output texture map bitstream.
  -o <output bitstream>             output total bitstream.
  
---------------------draco parameters---------------------------
  -point_cloud                      forces the input to be encoded as a point cloud.
  -draco_qp <value>                 quantization bits for the position attribute, default=11.
  -draco_qt <value>                 quantization bits for the texture coordinate attribute, default=10.
  -draco_qn <value>                 quantization bits for the normal vector attribute, default=8.
  -draco_qg <value>                 quantization bits for any generic attribute, default=8.
  -draco_cl <value>                 compression level [0-10], most=10, least=0, default=7.
  --skip ATTRIBUTE_NAME             skip a given attribute (NORMAL, TEX_COORD, GENERIC)
  --metadata                        use metadata to encode extra information in mesh files.
  -preserve_polygons                encode polygon info as an attribute.
Use negative quantization values to skip the specified attribute
---------------------hpm parameters---------------------------
  -hpm_cfg <hpm config>             hpm encode config path.
  -hpm_q <qp>                       hpm qp.
  -hpm_w <width>                    texture map width.
  -hpm_h <height>                   texture map height.
  -hpm_yuv <yuv file>               texture map yuv path. 
     

  # decode

  Main options:
  -h | -?                   show help.
  -i <input>               *input bitstream path.
  -oMesh <output>           output mesh file path.
  -oYuv <output>            output texture map yuv file path.
  -oMap <output>            output texture map png file path.

  # example
  -iMesh D:/airplane.obj -o D:/airplane.bin
  
  -iMesh D:/airplane.obj -iMap D:/airplane.jpg -hpm_cfg D:/MCEMv0.1/dependencies/hpm/cfg/encode_AI.cfg -hpm_q 63 -hpm_w 1024 -hpm_h 1024 -o D:/airplane.bin

  -iMesh D:/bag.ply -o D:/bag.bin
