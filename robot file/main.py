import tkinter as tk
from flexie import plotDistWorkspaceTraj, plotWorkspace, plotPathRobot, animatePathRobot


main_window = tk.Tk()
main_window.geometry('1000x600')

frame1=tk.Frame(main_window)                        #bg='grey'
frame1.grid(row=0, column=0,rowspan= 1,columnspan= 1 ,padx=10, pady=10, sticky='W', )

button1=tk.Button(frame1, text= 'dist workspace-traj'           ,command=lambda: plotDistWorkspaceTraj(main_window))
button1.grid(sticky='W',column=0, row=0,padx=10, pady=10)

button2=tk.Button(frame1, text= 'visualize workspace'           ,command=lambda: plotWorkspace(main_window))
button2.grid(sticky='W',column=1, row=0,padx=10, pady=10)

button3=tk.Button(frame1, text= 'plot robot following path'     ,command=lambda: plotPathRobot(main_window))
button3.grid(sticky='W',column=0, row=1,padx=10, pady=10)

button4=tk.Button(frame1, text= 'animate robot following path'  ,command=lambda: animatePathRobot(main_window))
button4.grid(sticky='W',column=1, row=1,padx=10, pady=10)

main_window.mainloop()
