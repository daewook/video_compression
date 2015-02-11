./TAppEncoderStaticd -c ../cfg/encoder_randomaccess_main.cfg -wdt 352 -hgt 288 -q 0 -fr 50 -f 5 --SEIDecodedPictureHash=1 -i ../../videos/bus_cif.yuv -b bus_cif.hevc -o HM_12_0_bus_cif.yuv 
#./TAppEncoderStatic -c ../cfg/encoder_intra_main.cfg -wdt 352 -hgt 288 -q 0 -fr 55 -f 10 --SEIDecodedPictureHash=1 -i ../../videos/bus_cif.yuv -b bus_cif.hevc -o HM_12_0_bus_cif.yuv 
./TAppDecoderStaticd -b bus_cif.hevc -o HM_12_0_bus_cif.yuv
