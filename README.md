# Logview - free hosted package

This is the deployment copy of Logview, a beginner-friendly ArduPilot DataFlash .bin analyzer that creates interactive plots and a detailed PDF flight-test report. It does not include any sample flight logs or generated reports.

## Free hosting limits

The hosted version accepts .bin logs up to **150 MB**. The local analyzer keeps its 3 GB setting. Streamlit Community Cloud processes uploaded logs on its infrastructure, and DataFlash logs may include GPS coordinates. Do not upload a log unless you are comfortable with that data flow. Ambient weather is not normally present in the log and is reported as unavailable rather than looked up from the GPS position.

## Deploy

1. Sign in to Streamlit Community Cloud with GitHub.
2. Choose this repository, branch main, and entrypoint app.py.
3. Choose logview as the custom subdomain if available; otherwise select another available name.
4. Deploy and check the app logs after its first launch.

Do not add flight logs, generated reports, or credentials to this repository.

## Dependencies

The deployment installs the packages in requirements.txt.
