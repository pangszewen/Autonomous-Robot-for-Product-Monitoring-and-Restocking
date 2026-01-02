import streamlit as st
import json
import time
import os

st.set_page_config(page_title="Object Pickup Monitor", layout="centered")

VIZ_DATA_FILE = "/tmp/robot_viz_data.json"

st.title("Object Pickup Monitor")

# Auto-refresh
refresh_rate = st.sidebar.slider("Refresh (seconds)", 0.1, 2.0, 0.5)

placeholder = st.empty()

while True:
    try:
        if os.path.exists(VIZ_DATA_FILE):
            with open(VIZ_DATA_FILE, 'r') as f:
                data = json.load(f)
            
            with placeholder.container():
                st.markdown(f"### Object Class: **{data.get('object_class', 'N/A')}**")
                st.markdown(f"### Distance: **{data.get('distance', 0):.2f} cm**")
                
                st.markdown("### Joint Angles:")
                col1, col2, col3 = st.columns(3)
                with col1:
                    st.metric("Theta 2", f"{data.get('theta2', 0):.2f}°")
                with col2:
                    st.metric("Theta 3", f"{data.get('theta3', 0):.2f}°")
                with col3:
                    st.metric("Theta 4", f"{data.get('theta4', 0):.2f}°")
                
                st.markdown(f"Positional Error: {data.get('positional_error', 0):.2f} cm")
                
                st.caption(f"Last update: {data.get('timestamp', 'N/A')}")
        else:
            with placeholder.container():
                st.info("Waiting for data...")
    
    except Exception as e:
        st.error(f"Error: {e}")
    
    time.sleep(refresh_rate)
    st.rerun()
