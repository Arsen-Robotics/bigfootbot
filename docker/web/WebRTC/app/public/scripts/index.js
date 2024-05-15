/**
 * Requests access to the user's media devices (video and audio) and streams. E.g. webcam and microphone.
 * If access is granted, sets the stream as the source object for the local video element.
 * If access is denied or an error occurs, logs a warning message.
 */
navigator.mediaDevices.getUserMedia({ video: true, audio: true })
  .then(stream => {
    const localVideo = document.getElementById("local-video");
    if (localVideo) {
      localVideo.srcObject = stream;
    }
  })
  .catch(error => {
    console.warn(error.message);
});
