#!/usr/bin/python
import rospy
import requests

from std_msgs.msg import String
from sensor_msgs.msg import Image
from final_result_msgs.msg import save_image
from message_filters import ApproximateTimeSynchronizer, Subscriber

rospy.init_node('chat_node')
pub_submit = rospy.Publisher('/target_detection', save_image, queue_size=10)
rate = rospy.Rate(10.0)

api = "https://api.openai.com/v1/chat/completions"
headers = {"Content-Type": "application/json", "Authorization": "Bearer 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"}

def chat(riddle_msg, image_msg):
    global key, api

    riddle = riddle_msg.data
    image = image_msg.data

    print("Get Here!")

    if len(riddle) == 0:
        return

    answer = None
    while(answer == None):
	payload = {"model": "gpt-3.5-turbo", "messages": [{"role": "user", "content": "Do not say anything but decimal integers above nine.\n\n" + riddle}], "temperature": 0}
	completion = requests.post(api, headers = headers, json = payload)
        response = completion.json()["choices"][0]["message"]["content"]
	answer = re.search(r'\d+', response).group()
        print(completion)
        answer = completion
        
        if answer != None:
            print("RIDDLE_SOLVED: ")
	    print(answer)
	    submission = save_image()
	    submission.class_id = answer
	    submission.save_img = image_msg.data
            pub_submit.publish(submission)
            break
	else:
	    print("FAILED!")
	    break
        
def listener(data):
    global images
    images.append(data)

if __name__ == '__main__':
    sync = ApproximateTimeSynchronizer([Subscriber('/qr_codes', String), 
                                        Subscriber('/qr', Image)],
                                       queue_size = 10, slop = 0.1)
    sync.registerCallback(chat)
    rospy.spin()
