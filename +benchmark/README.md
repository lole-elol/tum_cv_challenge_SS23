# Benchmarking

## Example commands to run benchmark
### Only 3D reconstruction
```matlab
benchmark.run("+benchmark/configs/benchmarkX", {"test/old_computer", "test/delivery_area_dslr_undistorted"}, "+benchmark/outputData", testDetection=false, testReconstruction=true)
```
### Only 3D model detection
```matlab
benchmark.run("+benchmark/configs/benchmarkX", "+benchmark/demoData", "+benchmark/outputData", testDetection=true, testReconstruction=false)
```
#### Both models (detection uses the outputs of reconstruction)
```matlab
benchmark.run("+benchmark/configs/benchmarkX", {"test/old_computer", "test/delivery_area_dslr_undistorted"}, "+benchmark/outputData", testDetection=true, testReconstruction=true)
```

## Example commands to display results
### Only 3D reconstruction
```matlab
benchmark.display("+benchmark/outputData", showDetection=false)
```
### Only 3D model detection
```matlab
benchmark.display("+benchmark/outputData", showReconstruction=false)
```
### Both
```matlab
benchmark.display("+benchmark/outputData")
```