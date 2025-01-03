from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError
import io


def SendToSlackMessage(message, username='PythonBot', icon_emoji=':thumbsup:',file=None):
    client = WebClient(token='')

    import matplotlib.pyplot as plt

    def plot_to_bytes():
        # Create a simple plot
        plt.figure()
        plt.plot([1, 2, 3, 4], [10, 20, 25, 30])
        plt.title('Sample Plot')

        # Save the plot to a bytes buffer
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        return buf

    file = plot_to_bytes()
    # Upload the image to Slack
    try:
        if file is not None:
            response = client.files_upload_v2(
                channel='C07EN5NGKDZ',
                file=file,
                title='progress',
                initial_comment=message
            )
        else:
            response = client.chat_postMessage(channel='C07EN5NGKDZ', text  = message, username = username, icon_emoji = icon_emoji)
    except SlackApiError as e:
        print(f"Error uploading file: {e.response['error']}")


if __name__ == '__main__':
    message = "Hello World!\nThis is a test message from Python."
    username = 'DatasetMaker'
    icon_emoji = ':learning:'
    SendToSlackMessage(message, username, icon_emoji)