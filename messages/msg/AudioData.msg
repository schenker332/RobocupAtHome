# Audio data message
uint32 sample_rate      # Sample rate in Hz (e.g., 44100)
uint16 channels         # Number of channels (1=mono, 2=stereo)
uint16 sample_width     # Bytes per sample (2 for 16-bit)
uint8[] data            # Raw audio data
