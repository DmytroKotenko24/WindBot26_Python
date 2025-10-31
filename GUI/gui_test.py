import tkinter as tk
from tkinter import messagebox

""" Simple GUI Application using Tkinter """

""" Functions """
#def on_click():
#    messagebox.showinfo("Hello", f"Hello, {entry.get()}!")

""" Window Setup """
root = tk.Tk() # Create the main window
root.title("WindBot'26") # Application Title
root.configure(bg="#1e1e1e") # Dark background
root.geometry("800x800") # Window size

""" Widgets """
#label = tk.Label(root, text="Enter your name:")
#label.pack(pady=5)
#entry = tk.Entry(root) # Input field
#entry.pack(pady=5) # Padding for better layout

button = tk.Button(root, text="Greet Me", command=on_click)
button.pack(pady=10)

root.mainloop()
