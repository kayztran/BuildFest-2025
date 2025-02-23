import numpy as np
import time
import asyncio
import threading
import cv2
import sounddevice as sd
import ffmpeg
import soundfile as sf
from datafeel.device import VibrationMode, discover_devices, LedMode, ThermalMode

# Set the MOV file path
VIDEO_FILE = "Final_video&audio.mov"
AUDIO_FILE = "Final_audio.wav"



#  Extract Audio from MOV using ffmpeg
def extract_audio_from_mov(video_file, output_audio):
    print("Extracting audio from MOV...")
    (
        ffmpeg.input(video_file)
        .output(output_audio, format="wav", acodec="pcm_s16le", ac=2, ar="44100")
        .run(overwrite_output=True, capture_stdout=True, capture_stderr=True)
    )
    print("Audio extracted successfully.")

#  Load Audio File
def load_audio(file_path):
    audio_data, sample_rate = sf.read(file_path, dtype="float32")
    channels = audio_data.shape[1] if len(audio_data.shape) > 1 else 1
    return audio_data, sample_rate, channels
    return

# Load DataFeel Dots (Using 3 devices)
devices = discover_devices(3)
if len(devices) < 3:
    print(f"Only found {len(devices)} DataFeel Dots. Need at least 3. Exiting...")
    exit(1)

dot_1, dot_2, dot_3 = devices[:3]  # Select the first 3 dots
print(f"Connected to DataFeel devices: {dot_1}, {dot_2}, {dot_3}")

# Define Actions at Specific Timestamps (seconds) for 3 Dots
ACTION_TIMESTAMPS = [
    {"time": 0.0, "led": (0, 0, 0), "vibration": (0, 0), "temp": 0.0},
    {"time": 19.0, "led": (0, 255, 0), "vibration": (0, 0), "temp": 0.30},
    {"time": 27.0, "led": (0, 255, 0), "pulse": (50, 0.25, 0.25, 0.5, 5), "temp": 0.30},
    {"time": 32.0, "led": (224, 44, 182), "vibration": (0, 0), "temp": 0.50},
    {"time": 36.0, "led": (224, 44, 182), "vibration": (0, 0), "temp": -0.75},
    {"time": 44.0, "led": (224, 44, 182), "vibration": (0, 0), "temp": -1},
    {"time": 49.0, "led": (0, 0, 0), "vibration": (0, 0), "temp": -1},
    {"time": 62.0, "led": (0, 0, 0), "pulse": (50, .5, 0.25, 0.5, 5), "temp": 0.25},
    {"time": 76.0, "led": (0, 0, 0), "vibration": (0, 0), "temp": 0.25},
    {"time": 88.0, "led": (248, 241, 35), "vibration": (0, 0), "temp": 0.35},
    {"time": 95.0, "led": (243, 255, 212), "pulse": (200, 1, 0.25, 0.25, 10), "temp": 0.45},
    {"time": 100.0, "led": (0, 0, 0), "vibration": (0, 0), "temp": 0.35},
    {"time": 115.0, "led": (216, 10, 252), "pulse": (200, 1, 0.25, 0.25, 10), "temp": 0.25},
    {"time": 127.0, "led": (0, 0, 0), "vibration": (0, 0), "temp": 0.0},
]

ACTION_TIMESTAMPS_2 = [
    {"time": 0.0, "led": (0, 0, 0), "vibration": (0, 0), "temp": 0.0},
    {"time": 76.0, "led": (247, 231, 126), "vibration": (0, 0), "temp": 0.0},
    {"time": 88.0, "led": (251, 171, 18), "vibration": (100, 0.25), "temp": 0.0},
    {"time": 95.0, "led": (255, 185, 32), "pulse": (300, 0.75, 0.25, 0.25, 10), "temp": 0.0},
    {"time": 100.0, "led": (216, 10, 252), "vibration": (0, 0), "temp": 0.0},
    {"time": 115.0, "led": (216, 10, 252), "pulse": (200, 1, 0.25, 0.25, 10), "temp": 0.0},
    {"time": 127.0, "led": (0, 0, 0), "vibration": (0, 0), "temp": 0.0},
]

#  Function to play audio asynchronously
def play_audio(audio_data, sample_rate, channels):
    print("Playing audio...")
    sd.play(audio_data, samplerate=sample_rate)
    sd.wait()
    print("Audio finished.")

def play_video(video_path):
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Could not open video file.")
        return

    # Get video frame rate (FPS)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_time = 1 / fps  # Time per frame in seconds

    while cap.isOpened():
        start_time = time.time()

        ret, frame = cap.read()
        if not ret:
            break  # Stop when video ends

        cv2.imshow("MOV Playback", frame)

        # Calculate remaining time to maintain correct playback speed
        elapsed_time = time.time() - start_time
        sleep_time = max(0, frame_time - elapsed_time)

        # Wait the required time for the next frame (keeping video at correct speed)
        time.sleep(sleep_time)

        # Check if user presses 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()



#  Function to handle haptic effects for each dot
async def apply_haptics(dot, action_timestamps):
    start_time = time.time()

    for action in action_timestamps:
        current_time = time.time() - start_time
        wait_time = action["time"] - current_time

        if wait_time > 0:
            await asyncio.sleep(wait_time)  # Wait until the correct timestamp

        # Apply LED Color
        r, g, b = action["led"]
        dot.set_led(r, g, b)



        if "pulse" in action:
            frequency, intensity, pulse_duration, pause_duration, cycles = action["pulse"]
            if intensity > 0:
                dot.registers.set_vibration_mode(VibrationMode.MANUAL)
                for _ in range(cycles):
                    dot.play_frequency(frequency, intensity)
                    await asyncio.sleep(pulse_duration)
                    dot.stop_vibration()
                    await asyncio.sleep(pause_duration)
        
        # Apply Normal Vibration if present
        elif "vibration" in action:
            frequency, intensity = action["vibration"]
            if intensity > 0:
                dot.registers.set_vibration_mode(VibrationMode.MANUAL)
                dot.play_frequency(frequency, intensity)
            else:
                dot.stop_vibration()
      
       
        # Apply Temperature
        dot.activate_thermal_intensity_control(action["temp"])

        print(f"Dot {dot}: Time {action['time']:.2f}s -> LED: {action['led']} | Temp: {action['temp']}")

    # Stop Vibration & Reset LEDs after the last action
    dot.stop_vibration()
    dot.set_led(0, 0, 0)
    print(f"Dot {dot}: Haptic sequence complete.")

# Main function to run all tasks concurrently
async def main():
    # Extract audio from MOV file
    extract_audio_from_mov(VIDEO_FILE, AUDIO_FILE)

    # Load audio file
    audio_data, sample_rate, channels = load_audio(AUDIO_FILE)

    # Start audio and video playback in separate threads
    audio_thread = threading.Thread(target=play_audio, args=(audio_data, sample_rate, channels))
    video_thread = threading.Thread(target=play_video, args=(VIDEO_FILE,))

    audio_thread.start()
    video_thread.start()

    # Run haptic effects on all 3 dots concurrently
    await asyncio.gather(
        apply_haptics(dot_1, ACTION_TIMESTAMPS_2),
        apply_haptics(dot_2, ACTION_TIMESTAMPS),
        apply_haptics(dot_3, ACTION_TIMESTAMPS),
    )

    # Wait for audio and video to finish
    audio_thread.join()
    video_thread.join()

# Run the updated script
if __name__ == "__main__":
    asyncio.run(main())
