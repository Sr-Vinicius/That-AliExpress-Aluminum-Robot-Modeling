## Model A robot's direct and inverse kinematics

Para a dedução analítica da cinemática inversa do manipulador robótico, é necessário, primeiramente representá-lo de acordo com a notação de Denavit-Hartenberg e deduzir a cinemática direta.
Para a representação matemática dos \textit{frames} do robô, usa-se a equação \ref{eq:dh}. 

<iframe src="https://onedrive.live.com/embed?resid=66DD28BFBFBDD6B5%2121496&authkey=!AHsslGz4tR0zPLU&em=2" width="476" height="288" frameborder="0" scrolling="no"></iframe>

Como as equações cinemáticas podem ficar extensas, termos como $cos\theta_1$ serão abreviados para apenas $C_1$. Além disso expressões como $(\theta_2 + \theta_3 + \theta_4)$ serão reduzidas para $\theta_{234}$. Nesse sentido, funções trigonométricas com somas de arcos como $\cos(\theta_2 + \theta_3)$, será, em sua forma compactada, somente $C_{23}$.

A matriz de transformação homogenia $^0T_1$ que representa a relação entre o \textit{frame} \{0\} e o \textit{frame} \{1\}, ou simplesmente $A_1$, de acordo com a fórmula \ref{eq:dh} será:

$$
A_1 = \left[\begin{array}{c c c c}
        C_1 & 0 & S_1 & 0\\
        S_1 & 0 & -C_1 & 0\\
        0 & 1 & 0 & l_4\\
        0 & 0 & 0 & 1
\end{array}\right]
$$

De forma análoga, para a representação de $A_2$, ou $^1T_2$, também se dá pela aplicação da relação de Denavit-Hartenberg:

$$
    A_2 = \left[\begin{array}{c c c c}
        C_2 & -S_2 & 0 & l_2 C_2\\
        S_2 & C_2 & 0 & l_2 S_2\\
        0 & 0 & 1 & 0\\
        0 & 0 & 0 & 1
    \end{array}\right]
$$

Semelhantemente, todas as transformações restantes $A_3$, $A_4$, $A_5$ são deduzidas pela equação \ref{eq:dh}.

$$
    A_3 = \left[\begin{array}{c c c c}
        C_3 & -S_3 & 0 & l_3 C_3\\
        S_3 & C_3 & 0 & l_3 S_3\\
        0 & 0 & 1 & 0\\
        0 & 0 & 0 & 1
    \end{array}\right]
$$


$$
    A_4 = \left[\begin{array}{c c c c}
        C_4 & 0 & -S_4 & l_4 C_4\\
        S_4 & 0 & C_4 & l_4 S_4\\
        0 & -1 & 0 & 0\\
        0 & 0 & 0 & 1
    \end{array}\right]
$$


$$
    A_5 = \left[\begin{array}{c c c c}
        C_5 & -S_5 & 0 & 0\\
        S_5 & C_5 & 0 & 0\\
        0 & 0 & 1 & l_5\\
        0 & 0 & 0 & 1
    \end{array}\right]
$$

Por fim, para obter a cinemática direta basta multiplicar as transformações, como demonstrado na equação \ref{eq:dkine}.


$$
    ^0T_5 = \left[\begin{array}{c c c c}
        C_{234} C_1 C_5 - S_1  S_5 & - C_5 S_1 - C_{234} C_1 S_5 & -S_{234} C_1 &  C_1[ L_3 C_{13} + L_2 C_2 + L_4 
       C_{234} - L_5 S_{234}]\\
        
        C_1 S_5 + C_{234} C_5 S_1 & C_1 C_5 - C_{234} S_1 S_5 & -S_{234} S_1 & S_1[ L_3 C_{13} + L_2 C_2 + L_4 C_{234} - L_5 S_{234}]\\
        
        S_{234} C_5 &  -S_{234} S_5 & C_{234} &  L_1 + L_3 S_{23} + L_2 S_2 + L_5 C_{234} + L_4 S_{234}\\
        
        0 & 0 & 0 & 1
    \end{array}\right]
$$

Para uma dada configuração articular, pode-se dizer que a ferramenta do manipulador possui uma posição e orientação tal que a transformação homogênea $^0T_5$ pode ser representada pela seguinte matriz:

$$
    ^0T_5 = \left[\begin{array}{c c c c}
        n_x & o_x & a_x & p_x\\
        n_y & o_y & a_y & p_y\\
        n_z & o_z & a_z & p_z\\
        0 & 0 & 0 & 1
    \end{array}\right]
$$

Durante o prosseguimento da dedução será convencionado que a equação \ref{eq:lhs} será o $LHS$ (\textit{Left-Hand Side}), e a equação \ref{eq:rhs} o $RHS$ (\textit{Right-Hand Side}).

O desenvolvimento da cinemática inversa, compreende encontrar o valor articular para um dado \textit{frame}. Tendo-se um valor da ferramenta desejado (LHS), e a equação de cinemática direta (RHS), é possível deduzir alguns valores igualando-se tais termos.

$$
    [LHS] = [RHS]
$$

Fazendo-se a razão dos elementos $a_{14}$ e $a_{24}$ das matrizes $[LHS]$ e $[RHS]$,

$$
    \begin{array}{c}
     \cfrac{p_y}{p_x} = \cfrac{S_1[L_3 C_{23} + L_2 C_2 + L_4 C_{234} - L_5 C_{234}]}{C_1[L_3 C_{23} + L_2 C_2 + L_4 C_{234} - L_5 C_{234}]}\\[0.4cm]
    \cfrac{p_y}{p_x} = \cfrac{\cos{\theta_1}}{\sin{\theta_1}}\\[0.4cm]
    \tan{\theta_1} = \cfrac{p_y}{p_x}
    \end{array}
$$

$$
    \theta_1 = \tan^{-1}\left( \cfrac{p_y}{p_x} \right)
$$

A equação \ref{eq:cintura} define o ângulo da "cintura" do manipulador robótico dado um \textit{frame} desejado na ferramenta.

O próximo ângulo capaz de ser obtido da igualdade entre $[LHS]$ e $[RHS]$ é o ângulo $\theta_5$. Ele vem da razão dos elementos $a_{31}$ e $a_{32}$ das matrizes.

$$
    \begin{array}{c}
        \cfrac{o_z}{n_z} = \cfrac{-S_{234} S_5}{S_{234} C_5}
    \end{array}
$$

Como seno é uma função par, pode-se remover o sinal da expressão.

$$
    \begin{array}{c}
        \cfrac{o_z}{n_z} = \cfrac{\sin{\theta_5}}{\cos{\theta_5}}\\[0.4cm]
        \tan{\theta_5} = \cfrac{o_z}{n_z}
    \end{array}
$$

$$
    \theta_5 = \tan^{-1} \left( \cfrac{o_z}{n_z} \right) 
$$

A partir desse ponto, não é possível obter mais ângulos articulares a partir da igualdade entre $[LHS]$ e $[RHS]$. Para isso serão necessárias manipulações algébricas que desacoplem ângulos no termo $[RHS]$. Uma alteração possível é multiplicar as matrizes \ref{eq:lhs} e \ref{eq:rhs} por $A_1^{-1}$. Isso irá remover senos e cossenos do ângulo $\theta_1$ que multipliquem outros termos angulares no $[RHS]$.

$$
    \begin{array}{c c c c}
        A_1^{-1}[LHS] = \left[\begin{array}{c c c c}
        n_x C_1 + n_y S_1 & o_x C_1 + o_y S_1 & a_x C_1 + a_y S_1 & p_x C_1 + p_y S_1\\
        n_z & o_z & a_z & p_z - L_1\\
        n_x S_1 - n_y C_1 & o_x S_1 - o_y C_1 & a_x S_1 - a_y C_1 & p_x S_1 - p_y C_1\\
        n_z L_1 & o_z L_1 & a_z L1 & p_z L_1 + 1
    \end{array}\right]
    \end{array}
$$

$$
    \begin{array}{c c c c}
        A_1^{-1}[RHS] = \left[\begin{array}{c c c c}
        C_{234}C_5 & -C_{234}S_5 & -S_{234} & L_3C_{23} + L_2C_2 + L_4C_{234} - L_5S_{234}\\
        S_{234}C_5 & -S_{234}S_5 & -C_{234} & L_3S_{23} + L_2S_2 + L_5C_{234} + L_4S_{234}\\
        -S_5 & -C_5 & 0 & 0\\
        0 & 0 & 0 & 1\\
    \end{array}\right]
    \end{array}
$$

$$
    A_1^{-1}[LHS] = A_1^{-1}[RHS] = A_2 A_3 A_4 A_5\\
$$

Com essa manipulação foi possível remover multiplicações entre senos e cossenos que acoplavam variáveis articulares. Os elementos $a_{13}$ e $a_{23}$ da igualdade \ref{eq:igualdade2} permitem isolar o somatório de ângulos $\theta_{234}$. Esse somatório será útil para o cálculo de uma variável articular assim que mais dois outros ângulos sejam encontrados. Ele também será necessário porque os ângulos seguintes possuem em sua fórmula termos como o seno e o cosseno de $\theta_{234}$.

$$
    \begin{array}{c}
    \cfrac{-\sin \theta_{234}}{\cos \theta_{234}} = \cfrac{a_x \cos \theta_1 + a_y \sin \theta_1}{a_z}
    \end{array}
$$

$$
    \begin{array}{c}
    \theta_{234} = \tan^{-1} \left( \cfrac{a_x \cos \theta_1 + a_y \sin \theta_1}{a_z} \right)
    \end{array}
$$

Dos elementos $a_{14}$ dos termos $A_1^{-1}[LHS]$ e $A_1^{-1}[RHS]$, consegue-se extrair a seguinte equação:

$$
    \begin{array}{c}
     p_xC_1 + p_y S_1 = L_3C_{23} + L_2C_2 + L_4 C_{234} - L_5S_{234}\\[0.2cm]
     p_xC_1 + p_y S_1 - L_4 C_{234} + L_5S_{234}  = L_3C_{23} + L_2C_2   
    \end{array}
$$

$$
     (p_xC_1 + p_y S_1 - L_4 C_{234} + L_5S_{234})^2  = L_3^2C_{23}^2 + 2 L_3C_{23} L_2C_2 + L_2^2C_2^2      
$$

Da mesma maneira, os elementos $a_{24}$ dos termos $A_1^{-1}[LHS]$ e $A_1^{-1}[RHS]$ dão a equação:

$$
    \begin{array}{c}
    pz - L_1 = L_3S_{23} + L_2S_2 + L_5C_{234} + L_4S_{234}\\[0.2cm]
    pz - L_1 - L_5C_{234} - L_4S_{234} = L_3S_{23} + L_2S_2
    \end{array}
$$

$$
     (pz - L_1 - L_5C_{234} - L_4S_{234})^2 = L_3^2S_{23}^2 + 2 L_3S_{23} L_2S_2 + L_2^2S_2^2 
$$

Somando as equações \ref{eq:theta3_1} e \ref{eq:theta3_2}, obtém-se a expressão;

$$
    \begin{array}{c}
        (pz - L_1 - L_5C_{234} - L_4S_{234})^2 + (p_xC_1 + p_y S_1 - L_4 C_{234} + L_5S_{234})^2 = L_3^2S_{23}^2 + 2 L_3S_{23} L_2S_2 \\[0.2cm]
        + L_2^2S_2^2 + L_3^2C_{23}^2 + 2 L_3C_{23} L_2C_2 + L_2^2C_2^2
    \end{array}
$$

A fim de reduzir ainda mais o tamanho das expressões durante o desenvolvimento algébrico, o termo do lado esquerdo da igualdade será chamado apenas de $A$.

No lado direito da igualdade, os termos relativos à comprimento de elos podem ser colocados em evidência.

$$
    A = L_3^2 (C_{23}^2+S_{23}^2) + L_2^2 (C_2^2 + S_2^2) + 2L_2L_3(C_{23}C_2 + S_{23}S_2) 
$$

Nos termos $C_{23}^2+S_{23}^2$ e $C_2^2 + S_2^2$ é possível aplicar a relação fundamental da trigonometria.

$$
    A = L_3^2 + L_2^2 + 2L_2L_3(C_{23}C_2 + S_{23}S_2) 
$$

Expandindo a abreviação aplicada nos senos e cossenos, fica evidente que o termo $C_{23}C_2 + S_{23}S_2$ é equivalente à relação trigonométrica do cosseno da subtração de dois arcos.

$$
    \begin{array}{c}
         A = L_3^2 + L_2^2 + 2L_2L_3(\cos{(\theta_2+\theta_3)}\cos\theta_2 + \sin{(\theta_2+\theta_3)}\sin\theta_2)\\[0.2cm]
         A = L_3^2 + L_2^2 + 2L_2L_3\cos{(\theta_2+\theta_3-\theta_2)}\\[0.2cm]
         A = L_3^2 + L_2^2 + 2L_2L_3\cos\theta_3\\[0.2cm]
         \cos\theta_3 = \cfrac{A -  L_3^2 - L_2^2}{2L_2L_3}
    \end{array}
$$

Expandindo novamente o termo $A$, chagamos à equação que representa o cosseno de $\theta_3$.

$$
    C_3 = \cfrac{(pz - L_1 - L_5C_{234} -L_4S_{234})^2 + (p_xC_1 + p_y S_1 - L_4 C_{234} + L_5S_{234})^2 -  L_3^2 - L_2^2}{2L_2L_3}
$$

Pela relação fundamental da trigonometria, sabe-se também o seno de $\theta_3$.

$$
    S_3 = \sqrt{1 - C_3}
$$

Finalmente, chega-se no valor do ângulo articular $\theta_3$ a partir do cálculo do arco tangente com os valores de \ref{eq:s3} e \ref{eq:c3}.

$$
    \theta_3 = \tan^{-1} \left( \cfrac{\sin \theta_3}{\cos \theta_3} \right)
$$

Para a dedução do $\theta_2$ também são manipulados os termos $a_{14}$ e $a_{24}$ das matrizes $A^{-1}[LHS]$ e $A^{-1}[RHS]$. O primeiro passo é desenvolver as somas de arco do seno e do cosseno de $(\theta_2 + \theta_3)$ na equação formada pelos elementos $a_{14}$.

$$
    \begin{array}{c}
         p_xC_1 + p_y S_1 = L_3C_{23} + L_2C_2 + L_4 C_{234} - L_5S_{234}\\[0.2cm]
         p_xC_1 + p_yS_1 = L_3 (C_2C_3 - S_2S_3) + L_2C_2 + L_4C_{234} - L5S_{234}\\[0.2cm]
         p_xC_1 + p_yS_1 = (L_3C_3)C_2 - (L_3S_3)S_2 + L_2C_2 + L_4C_{234} - L5S_{234}
    \end{array}
$$

$$
    (L_3C_3 + L_2)C_2 - (L_3S_3)S_2 = p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234}
$$

Pode-se fazer o mesmo com os elementos $a_{24}$ das matrizes.

$$
    \begin{array}{c}
        pz - L_1 = L_3S_{23} + L_2S_2 + L_5C_{234} + L_4S_{234}\\[0.2cm]
        pz - L_1 = L_3(S_2C_3 + S_3C_2) + L_2S_2 + L_5C_{234} + L_4S_{234}\\[0.2cm]
        pz - L_1 = (L_3C_3)S_2 + (L_3S_3)C_2 + L_2S_2 + L_5C_{234} + L_4S_{234}\\[0.2cm] 
    \end{array}
$$

$$
    (L_3C_3)C_2 + (L_3S_3 + L_2)S_2 = pz - L_1 - L_4S_{234} - L_5C_{234}
$$

As equações \ref{eq:a} e \ref{eq:b} formam um sistema de equações nas quais o seno $S_2$ e o cosseno $C_3$ são as variáveis. Isolando-se a variável $C_2$ da equação \ref{eq:a}:

$$
    C_2 = \cfrac{p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234} + (L_3C_3)S_2}{L_3C_3 + L_2}
$$

Substituindo $C_2$ na equação \ref{eq:b}:

$$
    \begin{array}{c}
        (L_3S_3) \left( \frac{p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234} + (L_3C_3)S_2}{L_3C_3 + L_2} \right) + (L_3C_3+L_2)S_2 = p_z - L_1 - L_4S_{234} - L_5C_{234}\\[1cm]
        
        (L_3S_3)(p_xC_1+p_yS_1-L_4C_{234}+L_5S_{234}) + (L_3S_3)^2S_2 + (L_3C_3+L_2)^2S_2 = \\[0.2cm] (pz - L_1 - L_2S_2 - L_5C_{234} - L_4S_{234})(L_3C_3+L_2)
    \end{array}
$$

$$
   \begin{array}{c}
       [(L_3S_3)^2+(L_3C_3+L_2)^2]S_2 = (pz - L_1 - L_2S_2 - L_5C_{234} - L_4S_{234})(L_3C_3+L_2)\\[0.2cm]
       - (L_3S_3)(p_xC_1+p_yS_1-L_4C_{234}+L_5S_{234})
   \end{array}
$$

Isolando a variável $S_2$ na equação \ref{eq:a}:

$$
    S_2 = \cfrac{-(p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234})+(L_3C_3+L_2)C_2}{(L_3S_3)}
$$

Substituindo $S_2$ \ref{eq:b}:

$$
    (L_3 S_3)C_2 + (L_3C_3 + L_2) \left( \tfrac{-(p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234})+(L_3C_3+L_2)C_2}{(L_3S_3)} \right) = p_z - L_1 - L_4S_{234} - L_5C_{234}
$$

$$
    \begin{array}{c}
         (L_3S_3)^2C_2 - (L_3C_3 + L_2)(p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234}) + (L_3C_3 + L_2)^2C_2\\[0.2cm]  = (pz-L_1-L_4S{234}-L_5{C_234})(L_3S_3)
    \end{array}
$$

$$
    \begin{array}{c}
             [(L_3S_3)^2 + (L_3C_3+L_2)^2]C_2 = (p_z-L_1-L_4S_{234}-L_5C_{234})(L_3S_3) \\[0.2cm] + (L_3C_3 + L_2)(p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234})
    \end{array}
$$

Fazendo a razão entre as equações \ref{eq:quase_theta2s} e \ref{eq:quase_theta2c} obtemos a tangente de $\theta_2$. Logo, pode-se calcular o arco tangente desse termo para chegar à expressão que representa o ângulo $\theta_2$.

$$
    \theta_2 = \tan^{-1} \left( \tfrac{(pz - L_1 - L_5C_{234} - L_4S_{234})(L_3C_3+L_2)
       - (L_3S_3)(p_xC_1+p_yS_1-L_4C_{234}+L_5S_{234})}{(p_z-L_1-L_4S_{234}-L_5C_{234})(L_3S_3) + (L_3C_3 + L_2)(p_xC_1 + p_yS_1 - L_4C_{234} + L_5S_{234})} \right)
$$

Com os valores de $\theta_2$, $\theta_3$ e $\theta_{234}$ já conhecidos, calcula-se o valor de $\theta_4$.

$$
    \theta_4 = \theta_{234} - \theta_2 - \theta_4
$$

Essas equações ainda não consideram os \emph{offsets} angulares nos ângulos $\theta_n$. \emph{Offsets} foram adicionados às articulações da cintura, ombro e pulso do robô de forma a obter um \emph{workspace} maior e com mais manipulabilidade. Além de um maior volume na área de trabalho, essa escolha de \emph{offset} permitiu ao robô manipular com mais flexibilidade objetos colocados em um plano horizontal abaixo de sua base. Esses \emph{offsets} devem ser subtraídos dos valores encontrados pelas equações de cinemática inversa. Para a cintura do robô foi adicionado um \emph{offset} de 30º, logo:

$$
    \theta_1' = \theta_1 - \cfrac{\pi}{6}
$$

Para o ombro do manipulador também foi adicionado um \emph{offset} de 30º.

$$
    \theta_2' = \theta_2 - \cfrac{\pi}{6}
$$

Para a articulação responsável pelo movimento de arfagem do pulso, foi adicionado um \emph{offset} de -90º.

$$
    \theta_4' = \theta_4 + \cfrac{\pi}{2}
$$


## MATLAB robot model and trajctory planning

<iframe src="https://onedrive.live.com/embed?resid=66DD28BFBFBDD6B5%2121500&authkey=!AGYzLi7t8pS3_Fw" width="98" height="120" frameborder="0" scrolling="no"></iframe>

<img src="https://onedrive.live.com/embed?resid=66DD28BFBFBDD6B5%2121497&authkey=%21AI5XjOX0tjdg9VI&width=560&height=420" width="560" height="420" />

<iframe src="https://onedrive.live.com/embed?resid=66DD28BFBFBDD6B5%2121499&authkey=!ABm4uS68ZLp_fCw" width="320" height="320" frameborder="0" scrolling="no" allowfullscreen></iframe>