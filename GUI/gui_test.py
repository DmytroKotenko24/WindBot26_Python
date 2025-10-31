import tkinter as tk

root = tk.Tk()
root.tk.call('tk', 'scaling', 1.0)  # Set scaling to default
root.title("WindBot'26")
root.configure(bg="#1e1e1e")
root.geometry("800x800")

def capturar_porta():
    pass

def capturar_torre():
    pass

def processar_porta():
    pass

def processar_torre():
    pass

""" Captura da Porta """
label1 = tk.Label(root, text="Iniciar Captura da Porta", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label1.pack(pady=5, fill='x')

button1 = tk.Button(root, text="Start", command=capturar_porta, font=("Arial", 24), bg="#333", fg="#fff")
button1.pack(pady=10)
""" --------------------------------------------------------------------------------------------------------- """

""" Captura da Torre """
label2 = tk.Label(root, text="Iniciar Captura da Torre", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label2.pack(pady=5, fill='x')

button1 = tk.Button(root, text="Start", command=capturar_porta, font=("Arial", 24), bg="#333", fg="#fff")
button1.pack(pady=10)
""" --------------------------------------------------------------------------------------------------------- """

""" Processamento da Porta """
label3 = tk.Label(root, text="Processar Porta", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label3.pack(pady=5, fill='x')

button1 = tk.Button(root, text="Start", command=capturar_porta, font=("Arial", 24), bg="#333", fg="#fff")
button1.pack(pady=10)
""" --------------------------------------------------------------------------------------------------------- """

""" Processamento da Torre """
label4 = tk.Label(root, text="Processar Torre", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label4.pack(pady=5, fill='x')

button1 = tk.Button(root, text="Start", command=capturar_porta, font=("Arial", 24), bg="#333", fg="#fff")
button1.pack(pady=10)
""" --------------------------------------------------------------------------------------------------------- """

""" Geração do Código RAPID """
label5 = tk.Label(root, text="Gerar Código RAPID", bg="#1e1e1e", fg="#ffffff", font=("Arial", 48), anchor='w')
label5.pack(pady=5, fill='x')

button1 = tk.Button(root, text="Start", command=capturar_porta, font=("Arial", 24), bg="#333", fg="#fff")
button1.pack(pady=10)
""" --------------------------------------------------------------------------------------------------------- """

root.mainloop()