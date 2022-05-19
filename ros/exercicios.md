# Exercícios ROS

1. Entendendo e usando comunicação por tópicos
    1. Usando as ferramentas apresentadas até agora (rosnode, rostopic, rqt), crie um nó que publique "Hello world" no tópico `/greetings`, e outro nó que ouça as mensagens desse tópico e as mostre no terminal.
    2. Reinicie os nós com os nomes de `chatter` e `listener`.
    3. Modifique `chatter` para repetir a mensagem constantemente a uma taxa de 1 Hz.
    4. Adicionar um novo publisher, `chatter2`, dizendo "Hello ROS" à mesma taxa.
    5. Analisar a estrutura de nós e tópicos usando CLI.
    6. Analisar a estrutura de nós e tópicos usando interface gráfica.

2. Aplicando comunicação por tópicos e serviços
    1. Entender como funciona o [turtlesim](http://wiki.ros.org/turtlesim). Verificar na documentação os nós, tópicos, parâmetros e serviços disponíveis.
    2. Inicializar o simulador e verificar o grafo de nós.
    3. Faça a tartaruga se mover sozinha em círculos.
    4. Inicializar teleoperação com o teclado e checar o grafo novamente.
    5. Limpar os caminhos marcados na tela.
    6. Adicionar uma nova tartaruga chamada 'turtle_bota' na posição (x, y, th) = (2, 2, 0).
    7. Faça turtle_bota teletransportar 2 metros para a frente.
    8. Encerrar todos os nós e o mestre.

3. Trabalhando com arquivos launch e parâmetros
    1. Criar um launch inicializando simultaneamente o simulador e o nó de teleoperação; terminar todos os nós e o mestre.
    2. Mudar a cor de fundo do simulador para totalmente verde.
    3. Adicionar um argumento para controlar a intensidade do brilho de fundo.
    4. Remapear o nó de teleoperação para controlar outra tartaruga, com o nome de "turtle_bota". Crie a nova tartaruga e inspecione o grafo de nós.


Soluções [aqui](solucoes.md).