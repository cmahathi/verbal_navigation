#!/usr/bin/env python

import rospy
from verbal_navigation.srv import *
from google.cloud import texttospeech
import os

def handle_wavenet_request(req):
    print "Text received: ", req.text
    # Instantiates a client
    client = texttospeech.TextToSpeechClient()

    # Set the text input to be synthesized
    synthesis_input = texttospeech.types.SynthesisInput(text=req.text)

    # Build the voice request, select the language code ("en-US") and the ssml
    # voice gender ("neutral")
    voice = texttospeech.types.VoiceSelectionParams(
        language_code='en-US',
        ssml_gender=texttospeech.enums.SsmlVoiceGender.NEUTRAL)

    # Select the type of audio file you want returned
    audio_config = texttospeech.types.AudioConfig(
        audio_encoding=texttospeech.enums.AudioEncoding.MP3)

    # Perform the text-to-speech request on the text input with the selected
    # voice parameters and audio file type
    response = client.synthesize_speech(synthesis_input, voice, audio_config)
    # The response's audio_content is binary.
    
    with open('output.mp3', 'wb') as out:
        # Write the response to the output file.
        out.write(response.audio_content)
        print('Audio content written to file "output.mp3"')

    print ("before speaking\n")

    os.system("cvlc output.mp3 && exit")
    print ("After speaking, before deleting")


    # os.remove("output.mp3")
    print ("About to return")
    return WavenetResponse(True)

def wavenet_server():
    rospy.init_node('wavenet_server')
    s = rospy.Service('wavenet', Wavenet, handle_wavenet_request)
    print "Ready to synthesize speech."
    rospy.spin()

if __name__ == "__main__":
    wavenet_server()
