from langchain.memory import ChatMessageHistory
from langchain_openai import ChatOpenAI
import os

# 初始化 ChatOpenAI 对象
chat = ChatOpenAI(api_key='sk-E4l5p0x0QcTVAF1N4839Fa3956F042Fb86E8624f478e1294', base_url="https://free.gpt.ge/v1/", temperature=0.7)

# 初始化 MessageHistory 对象
history = ChatMessageHistory()
history.add_user_message("你现在是一个医生，你需要判断我，也就是病人，到底是肠胃炎，感冒，还是发烧，这三种症状对应三种药，分别是肠胃炎药，感冒药，退烧药,一旦我的症状被你识别出来后，请你询问我是否开药,如果我同意，请你说“好的，为您拿xxx药（从三个药里面选一个出来）,")
# history.add_ai_message("")

while True:
    a = input("input: ")
    if "再见" in a:
        break
    history.add_user_message(a)
    ai_response = chat.invoke(history.messages)
    print(ai_response.content)
    if "好的，为您拿肠胃炎药" in ai_response.content:
        os.system("rosrun interbotix_demos moveit_ik_demo2.py 0 __ns:=wx250s")
    if "好的，为您拿感冒药" in ai_response.content:
        os.system("rosrun interbotix_demos moveit_ik_demo2.py 1 __ns:=wx250s")
    if "好的，为您拿退烧药" in ai_response.content:
        os.system("rosrun interbotix_demos moveit_ik_demo2.py 2 __ns:=wx250s")
    # word_to_voice(ai_response.content)
    history.add_ai_message(ai_response.content)
    # print(history.messages)
