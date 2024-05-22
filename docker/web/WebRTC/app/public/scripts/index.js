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


// 
this.io.on("connection", socket => { // .on listens for an event
  const existingSocket = this.activeSockets.find(
    existingSocket => existingSocket === socket.id
  );

  if (!existingSocket) {
    this.activeSockets.push(socket.id);

    socket.emit("update-user-list", { // .emit sends a message to the client that is connected
      users: this.activeSockets.filter(
        existingSocket => existingSocket !== socket.id
      )
    });

    socket.broadcast.emit("update-user-list", { // .broadcast.emit sends a message to all clients except the one that is connected
      users: [socket.id]
    });
  }
}
