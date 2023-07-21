# README - Docker + devcontainer for Odometry Project

_O conteúdo apresentado nesse documento só é estritamente válido para máquinas com sistema operacional Ubuntu 18.04 ou superior._

## 1 - Docker + devcontainer

- ### 1.1 - Docker

  O Docker é uma plataforma que permite desenvolver, testar e implantar aplicativos de forma rápida e fácil criando ambientes isolados e independentes.

  Funciona de maneira análoga à uma máquina virtual, porém com um consumo de recursos muito menor, pois não é necessário virtualizar um sistema operacional inteiro, apenas os recursos necessários para o funcionamento do aplicativo.

  - #### 1.1.1 - Images

    No Docker, uma imagem é um pacote de software autônomo e executável que deve incluir **tudo** o que é necessário para executar um ambiente de aplicativo, incluindo código, runtime, ferramentas do sistema, bibliotecas do sistema e configurações. As imagens são usadas para criar containers.

    Imagens podem ser criadas manualmente pelo usuário, automaticamente através de um Dockerfile ou importadas do Docker Hub.

  - #### 1.1.2 - Containers

    Um container é uma **instância em execução** de uma imagem Docker. Ele representa um ambiente isolado e seguro em que o aplicativo pode ser executado sem interferir em outros processos do sistema.
    Os containers devem ser executados independentemente do host, ou seja, devem ser portáveis e com pouca/nenhuma dependência do ambiente em que estão sendo executados.

    _No caso da imagem que será gerada nesse repositório, será necessário ceder permissões de acesso para que o container possa abrir janelas gráficas no host. Por isso, o docker é executado com a flag `--privileged` e com um script de inicialização salvo em `scripts/startupScript.sh`, que permite que o container tenha acesso a dispositivos do host, como o xhost._

  - #### 1.1.3 - Volumes

    Quando containers são destruídos, todo o ambiente criado após o seu "build" será **destruído** junto. Para isso, existem os volumes: volumes no Docker são mecanismos para persistência de dados.
    Eles permitem que os dados dentro de um container sejam armazenados fora do ciclo de vida do próprio contêiner, possibilitando o compartilhamento de dados entre o host e o container, bem como entre diferentes containers.
    Assim, podemos garantir que o trabalho realizado dentro do container não será perdido quando o mesmo for destruído.

    A extensão devcontainer do VSCode, como será explicado mais adiante, persiste não só os dados, mas também as configurações do ambiente de desenvolvimento.

  - #### 1.1.4 - Instalação e Teste

    Para instalar o Docker na sua máquina com Ubuntu 18.04 ou superior, siga as instruções abaixo:

    [Instruções de instalação do Docker no Ubuntu](https://docs.docker.com/desktop/install/linux-install/)

    Para testar a instalação do Docker, execute:

    ```bash
    docker --version
    ```

- ### 1.2 - devcontainer

  O devcontainer é uma extensão do VSCode que permite executar um ambiente Docker dentro da IDE, aproveitando o arsenal de extensões e capacidades de debug do VSCode. Antes de tudo, é necessário [Instalar o VSCode e a extensão](https://code.visualstudio.com/docs/devcontainers/tutorial).

  - #### 1.2.1 - devcontainer.json

    O arquivo devcontainer.json, encontrado em `/.devcontainer` é usado para definir as configurações do devcontainer. Ele contém informações como a imagem base do Docker, as extensões do VS Code a serem instaladas, variáveis de ambiente e outras configurações específicas do projeto.

  - #### 1.2.2 - Workspace

    O workspace é a pasta do projeto que será montada no container. Ele contém o código-fonte, os scripts e os arquivos de configuração do projeto. No caso, a pasta raiz desse repositorio é o workspace. Ao compilar o devcontainer, o workspace será montado no container em `/workspace/odometry`. É altamente recomendado que todo o desenvolvimento seja feito dentro do workspace, pois ele é persistido mesmo após a destruição do container e pode ser facilmente compartilhado via git.

  - #### 1.2.3 - Build

    Para buildar a imagem e executar o ambiente Docker dentro do devcontainer, execute os seguintes passos:

    1. Abra o VSCode na pasta raiz do projeto
    2. Abra o menu de comandos (Ctrl+Shift+P)
    3. Procure por "Reopen in Container" e execute o comando
    4. Aguarde o build do devcontainer
    5. Verifique o build executando no terminal da máquina host:

       ```bash
       docker images
       ```

       O nome da imagem deverá se parecer com `vsc-odometry...`

       ```bash
       docker ps
       ```

       Se o contâiner estiver em execução, deverá aparecer junto do seu nome.

    - **OBS**:
      1. _Cada compilação do contâiner recompilará todo o dockerfile, criando uma imagem diferente. Se o Dockerfile for alterado, essa compilação será necessária. Senão, basta a primeira vez, e o contâiner poderá ser reaberto com o comando "Reopen in Container" sem a necessidade de recompilação._
      2. _Caso haja alguma falha no processo de compilação (que, inclusive, é bem longo e pode demorar entre 40min a 2h, especialmente graças aos passos 3, 19 e 20) o Docker salva um cache com o último passo bem sucedido da compilação, então não precisa ter (muito) medo. Se o processo falhar, basta executar o comando "Reopen in Container" novamente e o Docker continuará a compilação de onde parou._
      3. _O processo de compilação, por padrão, é executado com 8 threads (flags -j8 no Dockerfile). Isso é feito para acelerar o processo de compilação, mas pode tornar o host praticamente inutilizável até o fim do build. Se isso acontecer e se for muito incômodo, basta diminuir o número de threads nas flags do Dockerfile para 4 ou 2._

# TODO

1. Terminar essa documentação
2. Estudar performance
3. Testar em outras máquinas
4. Partir para o docker da Jetson

# 2 - Odometria

- ## 2.1 - Scripts
  "Explicar o uso dos scripts utilizados para setar o Workspace de odometria"
- ## 2.2 - Softwares
  - ### 2.2.1 - ROS
    "Explicar o conceito de ROS"
  - ### 2.2.2 - Gazebo
    "Explicar o conceito de Gazebo"
  - ### 2.2.3 - RViz
    "Explicar o conceito de RViz"
  - ### 2.2.4 - RQT
    "Explicar o conceito de RQT"
  - ### 2.2.5 - QGroundControl
    "Explicar o conceito de QGroundControl"
  - ### 2.2.6 - MAVROS
    "Explicar o conceito de MAVROS"
  - ### 2.2.7 - VINS-Fusion
    "Explicar o conceito de VINS-Fusion"
