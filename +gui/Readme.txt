# README FOR TESTING GUI.mlapp

--> have `testModel.mat`, `testCloud.mat` in folder `+gui`

`testModel.mat`: the saved output of modelDetection (in `+logic`)
`testCloud.mat`: the pointcloud object (input of `modelDetection`)

--> go to folder `+gui`

--> RUN function with testsets (no need to give arguments, just run `gui.Gui`)

Window will appear: 2 options: 
- press Import button to upload images --> select folder with .jpg format files inside
(takes a while, if database is large)

- press button Reconstruct Room --> shows 3D room of test-data
   
