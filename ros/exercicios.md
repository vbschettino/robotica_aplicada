# Exercícios ROS

1. Entendendo comunicação por tópicos
    1. Usando as ferramentas apresentadas até agora (rosnode, rostopic, rqt), crie um nó que publique "Hello world" no tópico `/greetings`, e outro nó que ouça as mensagens desse tópico e as mostre no terminal.
    2. Reinicie os nós com os nomes de `chatter` e `listener`.
    3. Modifique `chatter` para repetir a mensagem constantemente a uma taxa de 1 Hz.
    4. Adicione um novo publisher, `chatter2`, dizendo "Hello ROS" à mesma taxa.
    5. Analise a estrutura de nós e tópicos usando CLI.
    6. Analise a estrutura de nós e tópicos usando interface gráfica.

2. Aplicando comunicação por tópicos e serviços
    1. Entenda como funciona o [turtlesim](http://wiki.ros.org/turtlesim). Verificar na documentação os nós, tópicos, parâmetros e serviços disponíveis.
    2. Inicialize o simulador e verificar o grafo de nós.
    3. Faça a tartaruga se mover sozinha em círculos.
    4. Inicialize teleoperação com o teclado e checar o grafo novamente.
    5. Limpe os caminhos marcados no simulador.
    6. Adicione uma nova tartaruga chamada 'turtle_bota' na posição (x, y, th) = (2, 2, 0).
    7. Faça turtle_bota teletransportar 2 metros para a frente.
    8. Encerre todos os nós e o mestre.

3. Arquivos launch e parâmetros
    1. Crie um launch inicializando simultaneamente o simulador e o nó de teleoperação; terminar todos os nós e o mestre.
    2. Mude a cor de fundo do simulador para totalmente verde.
    3. Adicione um argumento para controlar a intensidade do brilho de fundo.
    4. Remapeie o nó de teleoperação para controlar outra tartaruga, 'turtle_bota'. Crie a nova tartaruga e inspecione o grafo de nós.

4. Pacotes e rospy
    1. Crie um novo pacote, `first_pkg`. Esse pacote deve ter dependências em rospy e nas mensagens padrão do ROS (String).
    2. Usando rospy, crie um nó  que publique "Hello world1", "Hello world2","Hello world3"... no tópico `/greetings` a uma taxa de 1 Hz.
    3. Crie um nó que escute `/greetings` e, para cada mensagem recebida, mostre na tela "Eu ouvi: " + a mensagem.
    4. Crie um nó que receba mensagens em `/echo_in` e as republique em `/echo_out` mas repetidas. Ex: ao receber "Hello world", deverá mostrar na tela "Hello world...Hello world...Hello world...". O número de repetições deve ser controlável por um parâmetro.
    5. Crie um launch inicializando e conectando os 3 nós. Por padrão, o número de ecos a ser deve ser 3, mas controlável por um argumento do launch.

5. Usando recursos criados pela comunidade do ROS
    1. Use o mouse do seu computador para controlar a tartaruga do turtlesim.

Soluções [aqui](solucoes.md).
