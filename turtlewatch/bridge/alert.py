import os
import ssl
import certifi
import logging
from datetime import datetime, timedelta
from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError

logger = logging.getLogger("BridgeLogger")

class AlertSystem:
    last_alert_time = None
    COOLDOWN_SECONDS = 60  

    @classmethod
    def send_slack_message(cls, channel_id: str, message_text: str):
        if cls.last_alert_time:
            time_since_last = datetime.now() - cls.last_alert_time
            if time_since_last < timedelta(seconds=cls.COOLDOWN_SECONDS):
                wait_time = cls.COOLDOWN_SECONDS - time_since_last.total_seconds()
                return 

        # SSL Context setup
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
            
            cls.last_alert_time = datetime.now()
            
            logger.info(f"Message sent successfully: {result['ts']}")

        except SlackApiError as e:
            logger.error(f"Error sending message: {e.response['error']}")
