from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError
import io


def SendToSlackMessage(message, username='PythonBot', icon_emoji=':thumbsup:',file=None):
    client = WebClient(token='')

    # Upload the image to Slack
    try:
        response = client.files_upload_v2(
            channel='C07EN5NGKDZ',
            file=file if file is not None else io.BytesIO(b"Hello World!"),
            title='Sample Plot',
            initial_comment=message
        )
    except SlackApiError as e:
        print(f"Error uploading file: {e.response['error']}")
    # del sio
    # response=client.chat_postMessage(channel='C07EN5NGKDZ', text=message, username=username, icon_emoji=icon_emoji)


if __name__ == '__main__':
    message = "Hello World!\nThis is a test message from Python."
    username = 'DatasetMaker'
    icon_emoji = ':learning:'
    SendToSlackMessage(message, username, icon_emoji)