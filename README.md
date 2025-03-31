Start program

Ask user to choose input mode:
    - 0 = run simulator or load a test image
    - 1 = use real-time camera

  If inputMode == 0:
      load image from file
    
  Else If inputMode == 1:
      activate vision system
      acquire and save image frame
      set image from saved frame

convert image to grayscale (optional)
scale image (optional)
apply filters (low-pass, high-pass)
perform color separation to isolate robot
track robot position

Loop ends (or continue to next frame/image)
