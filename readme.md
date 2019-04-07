# Verbal Navigation


### Wavenet Service
Install text-to-speech:

`pip install --upgrade google-cloud-texttospeech`

Configure credentials (in terminal where wavenet will run):

`export GOOGLE_APPLICATION_CREDENTIALS="[workspace_directory]/src/verbal_navigation/credentials/bwi-text-to-speech-267d7b3ee8f0.json"`


### Run BWI_Guide
Run node:

`rosrun verbal_navigation BWI_Guide`

Run node:

`rosrun verbal_navigation Wavenet_Node.py`

Run robot subscriber node:

`rosrun verbal_navigation RobotPlanExecutor`


### Mapping Script
Run from directory verbal_navigation/src/multimap/[floor]

`rosrun bwi_planning_common logical_marker _map_file:=2.yaml _data_directory:=.`
