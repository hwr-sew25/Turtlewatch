import logging
import os
import ssl
import certifi 
from dotenv import load_dotenv
from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError

logger = logging.getLogger("BridgeLogger")

def send_slack_message(channel_id: str, message_text: str):
    """ Calling it send_slack_message("C09PRS9P08K", "Test with fixed SSL")"""
    ssl_context = ssl.create_default_context(cafile=certifi.where())
    client = WebClient(
        token=os.environ.get("SLACK_TOKEN"),
        ssl=ssl_context  
    )

    try:
        result = client.chat_postMessage(
            channel=channel_id, 
            text=message_text
        )
        logger.info(f"Message sent successfully: {result['ts']}")

    except SlackApiError as e:
        logger.error(f"Error sending message: {e.response['error']}")


