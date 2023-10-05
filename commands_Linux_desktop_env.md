### Desktop environment
- Display server is a program that manages graphical display output and input.  
- X windows system is a windowing system for bitmap displays, common on Unix-like operating systems.  
- "X11" refers to a version pf the X protocol. Xorg is an open source implementation of X.  
- Wayland is a comminication protocol that specifies the communication between a display server and its clients.  
  Wayland is a replacement for the X windows system.  
- To check which display server is running use `echo $XDG_SESSION_TYPE` (if output is x11 - Xorg is running,   
  if output is wayland - Wayland is running)  
- Display manager (e.g. GDM, LightDM) is a program that starts the display server, launches the desktop, and manages   
                 user authentication. Is not a part of the desktop environment.  
- `xinit` - allows a user to manually start an Xorg display server. It is used to initialize the Xorg display server   
            and provide a connection for remote X clients (e.g. `xinit /usr/bin/xfce4-session -- :1` - start Xorg display server   
            and launch Xfce4 desktop environment on display :1)  
- Session manager is a program that starts the desktop environment and manages its sessions (e.g. gnome-session, startxfce4)
- `startxfce4` - start Xfce4 desktop environment. startxfce4 is a script responsible for starting an Xfce session.  
               It runs xinit passing the file specified in the FILES subsection as an argument  
               You  may  want  to  modify  the  default  xinitrc  file.  In  order  to  do   that,   copy  
               /etc/xdg/xfce4/xinitrc to ~/.config/xfce4/xinitrc and modify that file.  
- Window manager is a program that draws the frames around windows and allows the user to move, resize and close them.  
               Is a part of the desktop environment.  
- Firstly display manager starts the display server, then session manager starts the desktop environment,  
  and then window manager starts the window manager.  

- VcXsrv is an open source display server for Microsoft Windows operating systems (Windows X-server based on the xorg git sources).  
  It allows you to start a display server on your Windows system and then connect to it from your Linux system  
  (This package allows a GUI to be started from a WSL distribution)  

- `xeyes` - show a pair of eyes that will follow the mouse pointer around the screen - this is a part of X11 utilities  

### TigerVNC and TightVNC
- TigerVNC adds encryption for all supported operating systems and not just Linux. Conversely, TightVNC has features that  
- TigerVNC doesn't have, such as file transfers
- `sudo apt install tigervnc-standalone-server` - install TigerVNC
- `sudo apt install tightvncserver` - install TightVNC
- `vncserver` - start VNC server
- `vncserver -kill :<display_number>` - stop VNC server
- `vncserver -list` - list all running VNC servers

- Lightweight Desktop to use with VNC server: Xfce4  
- `sudo apt install xfce4 xfce4-goodies` - install Xfce4