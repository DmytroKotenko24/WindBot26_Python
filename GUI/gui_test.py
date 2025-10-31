import tkinter as tk
import subprocess
import threading
import os

PADX = 10

# Get project root (parent of GUI folder)
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

root = tk.Tk()
root.tk.call('tk', 'scaling', 1.0)  # Set scaling to default
root.title("WindBot'26")
root.configure(bg="#1e1e1e")
root.geometry("800x800")
root.minsize(400, 400)  # Set minimal window size

def capturar_porta():
    progress_win = tk.Toplevel(root)
    progress_win.title("Capturing Port Progress")
    text = tk.Text(progress_win, height=20, width=80)
    text.pack()

    def run_script():
        process = subprocess.Popen(
            ["python3", "Capture_ScanPorta.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=os.path.join(project_root, "WindBot26")
        )
        for line in process.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)
        process.wait()
        text.insert(tk.END, "\nProcess completed.\n")
        text.see(tk.END)

    threading.Thread(target=run_script, daemon=True).start()

def capturar_torre():
    progress_win = tk.Toplevel(root)
    progress_win.title("Capturing Tower Progress")
    text = tk.Text(progress_win, height=20, width=80)
    text.pack()

    def run_script():
        process = subprocess.Popen(
            ["python3", "Capture_ScanTorre.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=os.path.join(project_root, "WindBot26")
        )
        for line in process.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)
        process.wait()
        text.insert(tk.END, "\nProcess completed.\n")
        text.see(tk.END)

    threading.Thread(target=run_script, daemon=True).start()

def processar_porta():
    progress_win = tk.Toplevel(root)
    progress_win.title("Processing Port Progress")
    text = tk.Text(progress_win, height=20, width=80)
    text.pack()

    def run_script():
        process = subprocess.Popen(
            ["python3", "OCVPorta_25.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=os.path.join(project_root, "WindBot26")
        )
        for line in process.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)
        process.wait()
        text.insert(tk.END, "\nProcess completed.\n")
        text.see(tk.END)

    threading.Thread(target=run_script, daemon=True).start()

def processar_torre():
    progress_win = tk.Toplevel(root)
    progress_win.title("Processing Tower Progress")
    text = tk.Text(progress_win, height=20, width=80)
    text.pack()

    def run_script():
        process = subprocess.Popen(
            ["python3", "OCVTorre_25.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=os.path.join(project_root, "WindBot26")
        )
        for line in process.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)
        process.wait()
        text.insert(tk.END, "\nProcess completed.\n")
        text.see(tk.END)

    threading.Thread(target=run_script, daemon=True).start()

def gerar_codigo_rapid():
    progress_win = tk.Toplevel(root)
    progress_win.title("Generating RAPID Code Progress")
    text = tk.Text(progress_win, height=20, width=80)
    text.pack()

    def run_script():
        process = subprocess.Popen(
            ["python3", "OCVCalculos_25.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=os.path.join(project_root, "WindBot26")
        )
        for line in process.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)
        process.wait()
        text.insert(tk.END, "\nProcess completed.\n")
        text.see(tk.END)

    threading.Thread(target=run_script, daemon=True).start()

""" Captura da Porta """
label1 = tk.Label(root, text="Iniciar Captura da Porta", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label1.pack(pady=5, fill='x', padx=PADX)

button1 = tk.Button(root, text="Start", command=capturar_porta, font=("Arial", 16), bg="#333", fg="#fff", anchor='w', width=8)
button1.pack(pady=2, anchor='w', padx=PADX)
""" --------------------------------------------------------------------------------------------------------- """

""" Captura da Torre """
label2 = tk.Label(root, text="Iniciar Captura da Torre", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label2.pack(pady=5, fill='x', padx=PADX)

button2 = tk.Button(root, text="Start", command=capturar_torre, font=("Arial", 16), bg="#333", fg="#fff", anchor='w', width=8)
button2.pack(pady=2, anchor='w', padx=PADX)
""" --------------------------------------------------------------------------------------------------------- """

""" Processamento da Porta """
label3 = tk.Label(root, text="Processar Porta", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label3.pack(pady=5, fill='x', padx=PADX)

button3 = tk.Button(root, text="Start", command=processar_porta, font=("Arial", 16), bg="#333", fg="#fff", anchor='w', width=8)
button3.pack(pady=2, anchor='w', padx=PADX)
""" --------------------------------------------------------------------------------------------------------- """

""" Processamento da Torre """
label4 = tk.Label(root, text="Processar Torre", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label4.pack(pady=5, fill='x', padx=PADX)

button4 = tk.Button(root, text="Start", command=processar_torre, font=("Arial", 16), bg="#333", fg="#fff", anchor='w', width=8)
button4.pack(pady=2, anchor='w', padx=PADX)
""" --------------------------------------------------------------------------------------------------------- """

""" Geração do Código RAPID """
label5 = tk.Label(root, text="Gerar Código RAPID", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label5.pack(pady=5, fill='x', padx=PADX)

button5 = tk.Button(root, text="Start", command=gerar_codigo_rapid, font=("Arial", 16), bg="#333", fg="#fff", anchor='w', width=8)
button5.pack(pady=2, anchor='w', padx=PADX)
""" --------------------------------------------------------------------------------------------------------- """

root.mainloop()