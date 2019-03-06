# Verbal Navigation
### Run path optimizer
Run node:
`rosrun verbal_navigation BWI_Guide`


### Wavenet Service
Install text-to-speech:
`pip install --upgrade google-cloud-texttospeech`

Configure credentials (in terminal where wavenet will run):
`export GOOGLE_APPLICATION_CREDENTIALS="[workspace_directory]/src/verbal_navigation/credentials/bwi-text-to-speech-267d7b3ee8f0.json"`

Run node:
`rosrun verbal_navigation Wavenet_Node.py`