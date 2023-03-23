# Soluções para [exercícios Python](exercicios.md)

1. Operações básicas no interpretador e variáveis
    ```python
    print("Hello world")
    print((23*60 + 34) * 60)
    x = 3
    y = 4.5
    w = False
    g1 = 'b'
    g2 = 'ocam'
    g3 = 'peao'
    print(type(x))
    print(type(y))
    print(type(w))
    print(type(g1))
    print(type(g2))
    print(type(g3))
    num = x * y
    print(type(num))
    ```

2. Listas
    ```python
    l1 = [1, 2, 3, 4, 5]
    l1[0]
    l1[:2]
    l1[2:]
    l1[-1]
    l1[-2:]
    l1.append(7)
    l1[-1] = 6
    l1[:2] = [0, 0]
    l2 = l1
    l3 = l1[:3]
    print(l1, l2, l3)
    l1[:2] = [1, 2]
    print(l1, l2, l3)
    ```

3. Tuples e dicionários
    ```python
    t1 = (-1, 2, 3, 4, 5)
    # t1[0] = 1 -> throws an error
    # Unlike lists, tuples are not mutable
    t2 = (['a', 'b', 'c'], 3, True)
    t2[0][:] = ['b', 'c', 'd']
    d1 = {'SP':12e6, 'MG':8e6, 'RJ':10e6, 'ES': 5e6}
    print(d1['MG'])
    d1['MG'] = 5e6
    ```

4. Controle de fluxo
    ```python
    words = ['cadeira', 'ventilador', 'mouse']
    vowels = ['a', 'e', 'i', 'o', 'u']
    for word in words:
        if word[-1] in vowels:
            print(f'{word} ends in {word[-1]}')
        else:
            print(f'{word} ends in a consonant')
    ```

5. Funções
    - Ver linhas 6 a 10 de [rectangle.py](rectangle.py)

6. Classes
    - [rectangle.py](rectangle.py) e [area_of_square.py](area_of_square.py)


7. Bibliotecas adicionais
    1. ```python
        import math
        print(math.sqrt(4))
       ```
    2. ```bash
        # Bash terminal (not python intepreter)
        pip install --user ipython numpy matplotlib
       ```
    3. [sin_plot.py](sin_plot.py)
