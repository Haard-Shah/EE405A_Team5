#!/usr/bin/python
import rospy
import requests
import re

from std_msgs.msg import String
from sensor_msgs.msg import Image
from final_result_msgs.msg import save_image
from message_filters import ApproximateTimeSynchronizer, Subscriber

rospy.init_node('chat_node')
pub_submit = rospy.Publisher('/target_detection', save_image, queue_size=10)
rate = rospy.Rate(10.0)

answered = []

api = "https://api.openai.com/v1/chat/completions"
headers = {"Content-Type": "application/json", "Authorization": "Bearer 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"}

def chat(riddle_msg, image_msg):
    global api, answered

    riddle = riddle_msg.data
    image = image_msg.data

    print(riddle)

    if len(riddle) == 0:
        return

    answer = None
    while(answer == None and not (riddle in answered)):
	payload = {"model": "gpt-3.5-turbo", "messages": [{"role": "user", "content": "Do not say anything but decimal integers above nine.\n\n" + riddle}], "temperature": 0}
	completion = requests.post(api, headers = headers, json = payload)
        response = completion.json()["choices"][0]["message"]["content"]
	checkme = re.search(r'\d+', response)
	if(checkme == None):
        #answer = response
	    return
	answer = checkme.group(0)
        
        if answer != None:
            print("RIDDLE_SOLVED: ")
	    print(answer)
	    submission = save_image()
	    submission.class_id = int(answer)
	    submission.save_img = image_msg
            pub_submit.publish(submission)
	    answered.append(riddle)
            break
	else:
	    print("FAILED!")
	    break
        
def listener(data):
    global images
    images.append(data)

if __name__ == '__main__':
    sync = ApproximateTimeSynchronizer([Subscriber('/qr_codes', String), 
                                        Subscriber('/camera/color/image_raw', Image)],
                                       queue_size = 10, slop = 0.1, allow_headerless = True)
    sync.registerCallback(chat)
    rospy.spin()
