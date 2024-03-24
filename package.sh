
target_path=$1

out_path=$target_path/RustPilot
mkdir -p $out_path
cp ./start_scripts/* $out_path/
cp ./mixers/* $out_path/
cp $target_path/rust_pilot $out_path

pushd $target_path
tar -czvf rust_pilot.tar.gz RustPilot
popd

