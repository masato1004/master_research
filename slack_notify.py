from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError


def SendToSlackMessage(message, username='PythonBot', icon_emoji=':thumbsup:'):
    client = WebClient(token='xoxb-3710074566071-8240371641203-0bloVcXEKlp7EgngU80OokB5') 
    response=client.chat_postMessage(channel='C07EN5NGKDZ', text=message, username=username, icon_emoji=icon_emoji)


if __name__ == '__main__':
    message = "Hello World!\nThis is a test message from Python."
    username = 'DatasetMaker'
    icon_emoji = ':learning:'
    SendToSlackMessage(message, username, icon_emoji)