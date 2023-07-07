from math import pi, atan2, sin, cos, ceil, floor, sqrt
from functools import lru_cache
from sys import setrecursionlimit
from time import process_time
from random import random, randrange, seed, vonmisesvariate
from xml.etree.ElementTree import PI
import matplotlib.pyplot as plt
import matplotlib as mpl

# enlève les marques dans la légende des graphiques
mpl.rcParams['legend.markerscale'] = 0
mpl.rcParams['legend.frameon'] = False

# fixe la profondeur max des récursions pour @lru_cache()
setrecursionlimit(10*8) # ne pas mettre trop, sinon "segmentation fault"

"""
DEFINITIONS

Un arbre enraciné T est une liste [ r, T_1, ..., T_k ] où r est la racine et T_i est le i-ème fils, lui-même un arbre enraciné. Donc [ r ] est le plus petit arbre possible, et [ r, [u], [v] ] est un arbre avec seulement deux fils (ici des feuilles u et v). Un arbre enraciné T sur une liste P de points est simplement un arbre enraciné dont les sommets sont des indices de P.
"""


#########################
#
# Fonctions de distances
#
#########################

def dist_L2(p,q):
    """
    Renvoie la distance euclidienne entre les points p et q, un point étant un couple ou une liste à deux éléments.
    """
    return sqrt( (p[0]-q[0])**2 + (p[1]-q[1])**2 )

def dist_L1(p,q):
    """
    Comme dist_L2(p,q) mais pour la distance L1.
    """
    return abs(p[0]-q[0]) + abs(p[1]-q[1])

def dist_Linf(p,q):
    """
    Comme dist_L2(p,q) mais pour la distance L∞.
    """
    return max(abs(p[0]-q[0]), abs(p[1]-q[1]))

def dist_Lp(p,q):
    """
    Comme dist_L2(p,q) mais pour la distance selon la norme L_p définie par LP_NORM.
    """
    return ( abs(p[0]-q[0])**LP_NORM + abs(p[1]-q[1])**LP_NORM )**(1/LP_NORM)

def dist_radius(p,q):
    """
    Comme dist_Lp(p,q) mais avec un rayon d'approximation défini par RADIUS. NB: Si RADIUS = 0, c'est exactement dist_Lp(p,q). Si RADIUS>0, alors il est possible que la distance dépasse le diamètre, que l'on borne par une variable locale 
    """
    if p == q: return RADIUS
    return min( dist_Lp(p,q) + 2*RADIUS, DIAMETER)

# @cache # plus rapide, mais à partir de Python 3.9
@lru_cache(maxsize=None)
def dist(i,j):
    """
    calcule la distance (DIST) entre les points (POINTS) d'indices i et j.
    """
    return DIST(POINTS[i],POINTS[j])

@lru_cache(maxsize=None)
def chord(t):
    """
    Longueur de la corde d'un cercle de rayon unité avec N>0 points placés régulièrement et liant le point d'indice 0 au point d'indice t. Sert essentiellement à segment_cm() et segment_c(). Normalement, c'est pareil que dist_L2( (1,0), (cos(t*2*pi/N),sin(t*2*pi/N)) ).
    """
    return 2 * sin(t * pi / N)


#############################################################
#
# Variables globales pour éviter des passages de paramètres
# et qui deviennent nécessaires dans les fonctions récursives
# avec @lru_cache().
#
#############################################################

N = 0 # nombre de points, généralement sans le robot réveillé, pour chord()
DIST = dist_L2 # fonction de distance par défaut, pour dist()
POINTS = []  # ensemble de points, pour dist()
ROOT = 0 # indice dans POINTS de la source, le robot initialement réveillé
UPPER_BOUND = -1 # borne supérieure utiliser dans les algos (peut dépendre de la fonction de distance, de l'excentricité de l'ensemble des points, de l'algorithme, ...)
SEED = None # sert pour récupérer la seed 
PROCESS_TIME = 0 # temps écoulé en seconde pour affichage dans draw_all
RADIUS = 0 # rayon d'approximation pour la distance dist_radius()
DIAMETER = 2 # diamètre par défaut pour la distance dist_radius()

##############################################
#
# Divers: eccentricity(), diameter(), depth()
# 
##############################################

def eccentricity(r=None, P=None, d=None):
    """
    Calcule l'excentricité du point P[r] dans P selon la fonction de distance d.

    Variables globales modifiées: POINTS, DIST (pour dist())
    """
    global POINTS, DIST # important pour dist()
    if r == None: r = ROOT # valeur par défaut ne marche pas sinon
    if P == None: P = POINTS # valeur par défaut ne marche pas sinon
    if d == None: d = DIST # valeur par défaut ne marche pas sinon
    POINTS, DIST = P, d # fixe les valeurs pour dist()

    x = 0 # valeur à renvoyer
    for v in range(len(P)): # pour tous les points
        x = max(x,dist(r,v))
    return x

def diameter(P=None, d=None):
    """
    Calcule le diamètre de la liste de points P selon la fonction de distance d. On ne suppose pas de symétrie pour d.

    Variables globales modifiées: POINTS, DIST (via eccentricity())
    """
    if P == None: P = POINTS # valeur par défaut ne marche pas sinon
    if d == None: d = DIST # valeur par défaut ne marche pas sinon

    x = 0 # valeur à renvoyer
    for r in range(len(P)): # pour tous les points
        x = max(x,eccentricity(r,P,d))
    return x

def depth(T, P=None, d=None):
    """
    Calcule la profondeur d'un arbre enraciné T sur P selon la fonction de distance d.

    Variables globales modifiées: POINTS, DIST (pour dist())
    """
    global POINTS, DIST # important pour dist()
    if P == None: P = POINTS # valeur par défaut ne marche pas
    if d == None: d = DIST # valeur par défaut ne marche pas
    POINTS, DIST = P, d # fixe les valeurs pour dist()

    if len(P) == 0 or len(T) <= 1: return 0
    r = T[0] # racine de T
    if r < 0 or r >= len(P): return 0 # point en dehors de P

    x = 0 # # valeur à renvoyer
    for i in range(1,len(T)): # pour chaque sous-arbre de T
        f = T[i] # i-ème fils de T
        x = max(x, dist(r,f[0]) + depth(f,P,d))
    return x


##########################
#
# Distributions de points
#
##########################

def init_seed(n=None):
    """
    Fixe ou pas la seed à n. Cela permet d'initialiser le générateur avec la valeur de la seed utilisée lors du dernier appel à init_seed. Attention ! Il ne faut pas appeler cette fonction trop souvent, puisqu'il ne peut y avoir que 10000 initialisations différentes.
    """
    global SEED
    SEED = randrange(10000) if n == None else n
    seed(SEED)

def generate_regular_polygon(n, r=1, c=(0,0)):
    """
    Génère une liste de n points placés régulièrement sur un cercle de rayon r et centré en c, c'est-à-dure un n-gone régulier. Les points sont ordonnés selon le sens direct.
    """
    P = [] # liste à renvoyer
    if n > 0: theta = 2*pi/n # utilisé que si n>0
    for i in range(n): # répéter n fois
        P += [ (c[0]+r*cos(i*theta), c[1]+r*sin(i*theta)) ]
    return P

def generate_von_mises(n, p=0.5, k=1, f=0):
    """
    Génère une liste de n points aléatoires dans le disque centré sur l'origine où la distance au centre vaut x^p avec x uniforme sur [0,1] et où l'angle suit une loi normale circulaire (plus exactement de von Mises) d'angle moyen a_i = i*2𝜋/k où i est un entier uniforme sur [0,k[. Il faut k >= 1. Le paramètre f est relié à l'écart type de l'angle par rapport à la direction a_i. Plus précisément, on pose comme écart type la valeur sigma = (a_{i+1} - a_i)/(2f) = 𝜋/(k*f). Donc plus f est grand, plus les angles sont concentrés en a_i. Puis, on s'arrange pour que l'écart type de la loi de von Mises corresponde à sigma. Pour cela on pose le paramètre de la loi kappa = 1/sigma^2, ce qui est une très bonne approximation (cf. https://dlwhittenbury.github.io/ds-1-sampling-and-visualising-the-von-mises-distribution.html).
    
    Si p=0, alors la distance est unitaire, et donc les points sont tous sur le cercle unité. Si f=0, alors la distribution de l'angle sera uniforme (variance infinie), quelque soit la valeur de k. Si f=+∞ (ou toute valeur f<0), alors la variance sera nulle (kappa=+∞) et les angles possibles seront uniquement des a_i. Mettre k=0 pour indiquer k=+∞, ce qui aura un effet similaire à poser f=0.
    
    Faire attention au fait que p=0.5 ne donne pas forcément une distribution uniforme sur la "fleur", c'est-à-dire le disque déformé par les k directions, en particulier lorsque k est petit et f assez grand.

    Ex:
    - generate_von_mises(n,0) -> pts aléatoires unif. sur le cercle
    - generate_von_mises(n,0.5) -> pts aléatoires unif. sur le disque
    - generate_von_mises(n,1,k,-1) -> pts aléatoires unif sur une étoile à k branches
    - generate_von_mises(n,0.1,k,3) -> pts aléatoires concentrés vers les k directions
    - generate_von_mises(n,-0.1,k,3) -> idem mais à l'extérieur du cercle

    Attention ! Les points ne sont pas généralement ordonnés selon l'angle à l'origine. Pour le faire il faut faire (par exemple):

        P = generate_von_mises(...)
        P.sort(key=lambda A:atan2(*A))

    """
    P = [] # liste à renvoyer
    if f < 0: kappa = float("inf") # une distribution uniforme (variance +∞)
    else: kappa = (k*f/pi)**2 # un écart type en 𝜋/(kf)
    if k == 0: k,kappa = 1,float("inf")

    for _ in range(n): # répéter n fois
        i = randrange(k) # une des k directions aléatoires
        a = vonmisesvariate(i*2*pi/k,kappa) # angle aléatoire autour de la direction choisie
        x = random()**p # distance aléatoire uniforme dans [0,1]
        P += [ (x*cos(a), x*sin(a)) ] # ajoute le nouveau point à la fin

    return P

def generate_convex(n):
    """
    Génère une liste de n points aléatoire en positions convexe. L'algorithme est en O(n*log(n)). Cela ne marche que pour la norme L2. Pour les autres normes il faudrait remplacer le tri selon l'angle (avec atan(y/x)) par une fonction/notion adéquate. On pourrait dire qu'un ensemble de points est en position convexe s'il existe un ordre des points tel que tout plus court chemin (selon la métrique d) entre u et v est à l'intérieur de la région délimité par cette liste ordonnée de points. Ceci dit, j'ai l'impression qu'un ensemble est convex est une notion indépendante de la métrique: s'il est convexe pour d, il l'est aussi pour d', car c'est lié à la notion d'intérieur.

    PRINCIPE.
    
        On part d'une liste de n points aléatoires dans [0,1[², puis on calcule la différence (donc le vecteur) entre deux points consécutifs. La somme des n vecteurs est nulle. On trie ces vecteurs selon l'angle, puis on dessine de proche en proche les points de l'enveloppe convexe (avec chaque fois un angle croissant donc).
    """
    P = []
    for _ in range(n): # n points aléatoires dans [0,1[²
        P += [(random(), random())]
    
    for i in range(n-1): # calcule les différences
        P[i] = (P[i][0] - P[(i+1)%n][0], P[i][1] - P[(i+1)%n][1])

    P.sort(key=lambda A: atan2(*A)) # tri selon les angles

    for i in range(1,n): # positions finales
        P[i] = (P[i][0] + P[i-1][0], P[i][1] + P[i-1][1])

    return P

def normalize_bc(P=None, d=None):
    """
    Modifie une liste de points P en ajoutant à sa fin leur barycentre et telle que l'excentricité depuis ce barycentre vaut 1. ROOT est modifiée pour qu'elle corresponde alors à ce barycentre. Les points de P sont d'abord recentrés pour que le barycentre soit placés à l'origine (0,0), puis les coordonnées sont mis à l'échelle pour que l'excentricité de ce point soit 1. Les distances relatives sont préservées si la fonction de distance d est linéaire, c'est-à-dire si d(t*A+C,t*B+C) = d(A,B) pour tout points A,B,C et réels t>0. C'est le cas des normes, comme L_p.

    Variables globales modifiées: POINTS et DIST à cause de eccentricity(), ROOT
    """
    if P == None: P = POINTS # valeur par défaut ne marche pas sinon
    if d == None: d = DIST # valeur par défaut ne marche pas sinon
    dist.cache_clear() # important pour eccentricity()

    # calcule le barycentre (sx,sy)
    n = len(P)
    if n == 0: return # rien à faire dans ce cas
    x = y = 0 # barycentre
    for i in range(n): # pour tous les points de P
        x,y = x+P[i][0], y+P[i][1]
    x,y = x/n, y/n
    global ROOT
    ROOT = n # modifie la racine
    P += [(x,y)] # ajoute le barycentre en fin de liste
    ex = eccentricity(ROOT,P,d) # calcule l'excentricité du barycentre
    if ex == 0: ex = 1 # dans ce cas, on met tous les points à (0,0)
    for i in range(n+1): # transforme aussi le barycentre
        P[i] = ((P[i][0]-x)/ex, (P[i][1]-y)/ex)
    return


############################################
#
# Fonctions de dessins: points, arbres, ...
#
############################################

def draw_points(P, c='gray', s=5, m='o'):
    """
    Dessine un ensemble de points P de forme m, de couleur c et de taille s dans le dessin plt courant. Pour un simple affichage des points, il faut à minima faire un plt.show() ou bien faire draw_all().
    """
    for (x,y) in P:
        plt.plot([x], [y], color = c, marker = m, ms = s, ls = 'none')

def draw_tree(T, P=None, c_edge='blue', w=0.005, arc=False, pts=True, c_pts='red', s=5, long=False, c_long='green'):
    """
    Dessine récursivement un arbre T enraciné sur une liste de points P.

    c_edge = couleur des arêtes
    w = épaisseur des arêtes
    arc = arc du père vers les fils (True) ou arête simple (False)
    pts = dessine les points de l'arbre (True) ou pas (False)
    c_pts = couleur des points (si pts=True)
    s = taille des points (si pts=True)
    long = dessine les arêtes/arcs de la (ou les) branche la plus longue (selon DIST)
    c_long = couleurs des branches les plus longues

    Variable globale modifiée (si long=True): POINTS (pour dist())
    """
    if long:
        global POINTS
        if P == None: P = POINTS
        POINTS = P
    if len(P) == 0 or len(T) == 0: return # rien à faire dans ces cas
    r = T[0] # racine
    if r<0 or r>len(P): return # indice en dehors de P
    r = P[r] # r = coordonnées de la racine
    if pts: # dessine la racine r
        plt.plot([r[0]], [r[1]], color = c_pts, marker = 'o', ms = s, ls = 'none')
    hl, hw = (0.1, 0.05) if arc else (0,0) # longueur et largeur des arcs
    if long: ex = depth(T) # excentricité de r

    for i in range(1,len(T)): # pour chaque sous-arbre

        # dessine récursivement le sous-arbre T[i]
        draw_tree(T[i],P,c_edge,w,arc,pts,c_pts,s,long,c_long)

        # dessine l'arc entre la racine r et son sous-arbre T[i]
        f = T[i][0] # racine du fils
        if f<0 or f>len(P): return # indice en dehors de P
        c = c_long if long and ex == dist(T[0],f) + depth(T[i]) else c_edge # si branche longue
        f = P[f] # coordonnées de f
        lss, col = (':','orange') if T[0] == ROOT else ('-',c) # pointillé pour la 1ère arête
        plt.arrow(r[0], r[1], f[0]-r[0], f[1]-r[1], color = col, width = w, head_width = hw, head_length = hl, length_includes_head = True, ls = lss, overhang = .2)

def draw_all(title=None, x=None, T=None, P=None, d=None):
    """
    Dessine et affiche une solution x ainsi qu'un arbre enraciné T sur un ensemble de points P, title étant juste un commentaire. Calcule et affiche aussi la profondeur de T selon la fonction de distance d. Un simple appel à draw_all() affiche les points de POINTS, son excentricité et son diamètre. En posant ROOT=None, on empêche de distinguer la racine des autres points et le calcule le son l'excentricité. Attention ! PROCESS_TIME est remis à zéro. Pour ne pas afficher la ligne avec 'time', mettre PROCESS_TIME = 0. Pour ne pas afficher la ligne 'seed', mettre SEED = -1.

    Variables globales modifiées: POINTS, DIST (via eccentricity()), PROCESS_TIME
    """
    global PROCESS_TIME
    if P == None: P = POINTS # valeur par défaut ne marche pas sinon
    if d == None: d = DIST # valeur par défaut ne marche pas sinon
    dist.cache_clear() # important pour dist(), eccentricity() et diameter()

    n = len(P) # nombre de points
    s = max(1,6 - floor(n/50)) # taille variables des points
    w = s/1000 # épaisseur variable des arêtes
    arc = True if n < 60 else False # orientation ou pas des arêtes ?
    r = ROOT if T == None else T[0] # racine, éventuellement de l'arbre s'il existe

    plt.clf() # efface le dessin
    plt.axis('off') # enlève les axes
    plt.axis('equal') # aspect ratio: pour avoir des cercles en forme de cercle

    # NB: les objets sont affichés les uns au-dessus des autres, donc on dessine la racine en dernier pour la voir même s'il y a beaucoup de points

    draw_points(P,'gray',s) # dessine les points en gris et de taille s
    if T != None: # si l'arbre existe ...
        draw_tree(T,P,'blue',w,arc,True,'red',s,True) # dessine l'arbre
    if ROOT != None: # dessine la racine
        draw_points([P[r]],'orange',1.8*s,'X') # dessine la racine

    # calcule la légende à afficher 
    L = []; t = ""
    if title != None: t += title
    if x != None: t += f" = {x:.3f}"
    if t != "": L += [t]
    if T != None: L += [f"depth = {depth(T,P,d):.3f}"]
    L += [f"#pts = {n}"]
    L += [f"diam = {diameter(P,d):.3f}"]
    if ROOT != None: L += [f"ecc = {eccentricity(r,P,d):.3f}"]
    if PROCESS_TIME > 0:
        L += [f"time = {PROCESS_TIME:.3f}s"]
        PROCESS_TIME = 0 # pour éviter de ré-afficher le même temps encore et toujours
    if SEED >= 0: L += [f"seed = {SEED}"]

    plt.figlegend(L) # ajoute la légende L
    plt.show() # affiche le dessin
    return


##############################################
#
# ALGORITHMES:
#
#  - SEGMENT_CM(n)
#  - SEGMENT_C(n)
#  - SEGMENT(P,d)
#  - GREEDY_SEGMENT(P,d)
#
##############################################

"""
Les algorithmes SEGMENT_CM et SEGMENT_C dérivent de l'algorithme général SEGMENT qui donnent des arbres optimaux parmi ceux d'une certaine classe. Ils sont dédiés et optimisés pour les polygones réguliers (C = circle et M = monotone, on ne revient pas en arrière). L'algorithme SEGMENT est plus général et s'applique à n'importe quelle liste ordonnée de points et n'importe quelle fonction de distance (a priori métrique pour accélérer un peu, mais on pourrait s'en passer, c'est indiqué dans le code où l'on s'en sert). On peut retrouver le résultat de SEGMENT_C directement avec SEGMENT et une liste de points adéquate, mais c'est plus long.

L'algorithme SEGMENT donne l'arbre optimal de sorte que les points de chaque sous-arbre forment un ensemble de points consécutifs dans la liste ordonnée de points donnée en paramètre. Il est en fait un cas particulier de l'algorithme OPTIMAL dans le cas où un ordre cyclique doit être respecté. L'algorithme est en O(n^8) une fois données cette liste et la fonction de distance. On remarque que si les points sont en position convexe, alors l'arbre produit par SEGMENT (et ses variantes) est nécessairement planaire (sauf peut-être pour la première arête car, avec source, les points ne forment plus un ensemble convexe). Inversement, si les points sont en position convexe, SEGMENT donnera la meilleure solution planaire possible (sauf possiblement la première arête). Malheureusement, il existe des points en position convexe où la solution optimale n'est pas planaire (en dehors de la 1ère arête). Une propriété intéressante de cet algorithme est la suivante:

PROPOSITION (optimalité de SEGMENT). Pour tout ensemble de points, il existe un ordre sur P tel que SEGMENT(P) donne la solution optimale.

Donc pour décrire une solution optimale, quelle que soit la métrique, il suffit de décrire une permutation des points, l'arbre étant implicite et pouvant être retrouvé en temps polynomial. Cela vient du fait que pour tout arbre optimal, l'ordre DFS donnera bien sûr un arbre étiqueté de la classe où SEGMENT cherche l'optimal. Notons en passant que le nombre d'arbres de cette classe est environ 2^n (il y a au plus deux intervalles possibles si l'arbre à deux fils, qui récursivement fait 2^n étiquetages possibles). La proposition ci-dessus justifie également l'algorithme GREEDY_SEGMENT qui, à l'instar du TSP, essaye de trouver un bon ordre pour P.

NB1. Est-il possible de relier le résultat de SEGMENT à la longueur de la tournée générée POINTS ? Existe-il un algorithme polynomial pour générer un TSP assez court sur un ensemble de POINTS de rayon unité ? Malheureusement la longueur de la tournée optimale n'est pas constante même pour des points de rayon unité. En effet, le théorème de Beardwood-Halton-Hammersley dit que pour des points aléatoires uniformes le carré [0,1]², le TSP est en b*sqrt(n) où b est une constante ~ 0.74.

NB2. On à l'impression (à prouver) qu'à longueur de tournée fixée (ou longueur de chemin), disons L, le temps pour SEGMENT sera maximisé lorsque la distance moyenne entre les points est maximisée, soit sur le cercle. C'est pas très clair car un mauvais exemple est lorsque les points sont alignés avec un point de départ au mileu et tous les points situés à distance L/2 de part et d'autres de s.

Pour revenir aux points en position convexe, on peut quand même se demander si une généralisation de SEGMENT relativement simple (cf. ci-après) ne permettrait pas d'avoir un algorithme polynomial. On s'intéresse aux arbres tels que les points de chaque sous-arbre forment un ensemble d'au plus k segments consécutifs de points dans la liste. Notons MULTI_SEGMENT(k,P,d) la généralisation de SEGMENT(P,d), ce dernier algorithme traitant du cas k = 1. Il faut remarquer que MULTI_SEGMENT peut s'implémenter en n^O(k), l'idée étant que les ensembles correspondant à chaque sous-arbres peuvent se décrire avec 2k entiers de log(n) bits. Le nombre d'appels va donc être en n^{2k+O(1)} en utilisant une bonne table de hachage. Bref, la seule question pertinente est de savoir si une solution optimale peut avoir beaucoup de trous dans ses sous-arbres. On ne voit pas trop l'intérêt d'avoir une solution optimale avec plus qu'un trou par exemple. Une autre façon de voir est de se demander s'il est possible dans une solution optimale d'avoir des branches (ou plutôt si on ne peut pas toujours éviter d'avoir des branches) qui font beaucoup de zigzags entre les parois du convexe. La notion de zigzags est un peu flou, mais on voit l'idée: étant donné une arête u-v, le convexe est coupé en deux parties (convexes). Peut-il avoir une branche disons issue de v qui passe d'une partie à l'autre un nombre non constant de fois, c'est-à-dire qui coupe l'arête u-v autant de fois que souhaitée ? L'inégalité triangulaire pourrait l'empêcher. Si oui, cela resterait donc vrai aussi pour toute métrique, car il me semble que la notion d'ensemble convexe est indépendant de la métrique (cf. generate_convex()).

Une autre généralisation est de paramétrer le niveau de convexité de la liste ordonnée. En gros, lorsque l'ensemble est convexe, la tangente qui suit le convexe est monotone (croissante si on tourne dans le sens direct). Disons qu'elle varie de -x à +x. Mais on pourrait imaginer des variations plus complexes de -x à +x en passant par b pics (ou vallées, b=0 si convexe). L'intuition est qu'il devrait toujours être possible d'avoir un solution optimale où le nombre de segments est une fonction du paramètre b, ce qui serait une super chouette généralisation du cas convexe qu'on a pas prouvé. Une paramétrisation possible est le nombre de couches d'un ensemble de points, obtenu en épluchant l'ensemble successivement par enveloppe convexe.
"""

@lru_cache(maxsize=None)
def segment_cm(k):
    """
    On suppose qu'on est sur un cercle avec N >= 3 points placés régulièrement et qu'un point du cercle possède deux robots réveillés. La fonction renvoie alors le temps pour que ces deux robots réveillent les k >= 0 autres robots suivants selon la stratégie suivante: un des deux robots se charge du suivant tant dit que l'autre fait un saut de longueur t >= 2, paramètre t qu'il faut optimiser dans [2,k]. Le temps renvoyé est le max pris par les deux robots et minimiser sur t. Les robots réveillés recommencent alors récursivement la stratégie.

    La fonction renvoie également un arbre relatif du type [0], [1, [0]] ou [1, T1, t, T2], à convertir avec convert_tree(), où le paramètre t est la valeur des cordes optimales trouvées et les où les sous-arbres T1, T2 correspondent au 1er et 2e fils, le 1er fils étant placé avant le 2e dans l'ordre des points.

    L'algorithme est en O(k^2) car: (1) le coût de la fonction sans les appels récursifs est O(k); et (2) grâce à la mémorisation, il n'y a que O(k) appels différents.
    """
    if k == 0: return 0, [0]  # cas d'une feuille
    if k == 1: return chord(1), [1, [0]]  # cas d'un seul fils
    m = UPPER_BOUND # majorant sur la valeur retournée
    
    for t in range(2, k+1): # cherche la meilleure corde t dans [2,k]
        x1,T1 = segment_cm(t-2) # branche pour la corde 1
        x2,T2 = segment_cm(k-t) # branche pour la corde t>1
        x = max(chord(1) + x1, chord(t) + x2)
        if x < m: m,tmin,T1min,T2min = x,t,T1,T2 # on a fait mieux

    return m, [1, T1min, tmin, T2min]

def SEGMENT_CM(n=None):
    """
    On suppose n >= 0 robots placés régulièrement sur un cercle et un robot au centre qui est réveillé. La fonction renvoie la profondeur du meilleur arbre tel que les deux fils sont toujours placés après le père (sauf le premier robot informé du cercle). L'arbre produit est nécessairement planaire sauf la 1ère arête (propriété héritée de SEGMENT). Elle renvoie aussi l'arbre relatif (cf. segment_cm()) enraciné à partir du 2e point informé du cercle. C'est donc un sous-arbre de la solution qui n'existe pas si n = 0 ou 1. Attention ! La fonction fixe N = n ce qui est important pour chord().

    STRATÉGIE. On part du centre vers le point 0 du cercle, puis chacun des deux robots réveillent son voisin. À partir de là, on applique la stratégie récursive segment_cm(k,n) pour les k = ceil((n-3)/2) points restant dans la plus grande moitié. On suppose n >= 0, mais les cas n = 0,1,2 sont gérés à la main. La complexité est celle de segment_cm(n/2), soit O(n^2).

    Variable globale modifiée: N, UPPER_BOUND, PROCESS_TIME.
    """
    global N, UPPER_BOUND, PROCESS_TIME
    if n == None: n = N # valeur par défaut ne marche pas sinon
    N = n # fixe la variable globale pour chord()
    UPPER_BOUND = 3 # majorant sur segment_cm()
    segment_cm.cache_clear() # efface cache, sinon résultats incorrects
    chord.cache_clear() # efface cache, sinon résultats incorrects

    if n == 0: return 0, None # pas d'arbre dans ce cas
    if n == 1: return 1, None # un seul point à réveiller
    if n == 2: return 1 + chord(1), [0] # deux points à réveiller
    k = ceil((n-3)/2) # k points à réveiller sur la +grande moitié

    PROCESS_TIME = process_time()
    x,T = segment_cm(k) # NB: k>=0, car n>=3
    PROCESS_TIME = process_time() - PROCESS_TIME

    return 1 + chord(1) + x, T

@lru_cache(maxsize=None)
def segment_c(k1,k2):
    """
    On suppose N > 0 points placés régulièrement sur un cercle et qu'un point possède deux robots réveillés. La fonction renvoie le temps pour que ces deux robots réveillent les k1 >= 0 robots précédents et les k2 >= 0 robots suivants selon la stratégie suivante: le 1er robot se charge des k1 robots précédents et le 2e robot se charge des k2 suivants, avec les meilleures cordes possibles en recommençant récursivement. Si k1 = 0 (ou k2 = 0), alors les deux robots se partagent au mieux le réveil des k2 (ou k1) robots restant.

    La fonction renvoie également un arbre du type [0], [t, [0] ou [ t1, T1, t2, T2 ] donnant les cordes t, t1, t2 optimales pour informer les k1 points avant (ainsi que l'arbre récursif T1 correspondant) et les k2 points après (ainsi que l'arbre récursif T2 correspondant). Les cordes sont signées pour savoir si elles partent vers l'avant ou l'après du point initial avec les deux robots. Donc en général, t1 < 0 et t2 > 0. Mais si k1 ou k2 est nul, t1 et t2 peuvent être du même signe.

    C'est une simple adaptation de SEGMENT() consistant à remplacer dist() par chord() et à supprimer le paramètre racine. L'algorithme est en O( k1^2 * k2^2 * |k1-k2| ) car: (1) le coût de la fonction sans les appels récursifs est O( k1 * k2 * |k1-k2| ); et (2) grâce à la mémorisation, il n'y a que O( k1 * k2 ) appels différents.
    """
    if k1 == k2 == 0: return 0, [0] # on s'arrête dans ce cas

    # CAS 1: k1,k2>0. Il faut trouver la meilleur corde de chaque coté
    # (en parallèle), soit le meilleur t dans [1,k1], puis dans [1,k2]
    if k1>0 and k2>0:
        x1min = x2min = UPPER_BOUND # majorant sur la valeur retournée
        for t in range(1, k1+1): # meilleure corde avant r dans [1,k1]
            x,T = segment_c(k1-t, t-1); x += chord(t)
            if x < x1min: x1min, t1min, T1min = x, t, T
        for t in range(1, k2+1): # meilleure corde après r dans [1,k2]
            x,T = segment_c(t-1, k2-t); x += chord(t)
            if x < x2min: x2min, t2min, T2min = x, t, T
        return max(x1min, x2min), [-t1min, T1min, t2min, T2min] # le pire des deux branches

    # CAS 2: k1=0 ou k2=0. Les cordes t1,t2 partent toutes deux du même coté:
    # avant r si k2=0 ou après r si k1=0. Il faut donc réveiller les robots
    # à l'aide de deux robots et donc trouver 2 cordes t1<t2 tout et en
    # découpant l'intervalle ]t1,t2[

    # on s'arrange pour avoir k1=0 et k2>0
    s = +1 # signe: +1 si après (k1=0), -1 si avant (k2=0)
    if k1 > 0: # donc k2=0
        k1,k2 = k2,k1 # échange k1 et k2
        s = -1

    # CAS 2.1: k2=1, un fils
    if k2 == 1:
        return chord(1), [s, [0]] # cordes +1 ou -1

    # CAS 2.2: k2>=2, deux fils
    m = UPPER_BOUND # majorant sur la valeur retournée
    for t1 in range(1, k2):  # t1=1..k2-1, NB: k2>=2
        c1 = chord(t1)
        for t2 in range(t1+1, k2+1):  # t2=t1+1..k2 (t2=t1 inutile si métrique)
            c2 = chord(t2)
            k = t2-t1-1 # il y a k>=0 robots à se partager entre t1 et t2
            for i in range(0, k+1): # on en prend i pour t1 et k-i pour t2
                x1,T1 = segment_c(t1-1, i) if s>0 else segment_c(i, t1-1)
                x2,T2 = segment_c(k-i, k2-t2) if s>0 else segment_c(k2-t2, k-i)
                x = max(c1+x1, c2+x2)
                if x < m: m,t1min,T1min,t2min,T2min = x,t1,T1,t2,T2 # on a trouvé mieux
    return m, [s*t1min, T1min, s*t2min, T2min] # cordes de même signe

def SEGMENT_C(n=None):
    """
    On suppose n >= 0 robots placés régulièrement sur un cercle et un robot au centre qui est réveillé. La fonction renvoie la profondeur du meilleur arbre planaire. Elle renvoie aussi l'arbre relatif [0], [t, [0]] ou [t1, T1, t2, T2], à convertir avec convert_tree(), où les paramètres t, t1 et t2 sont la valeur des cordes optimales trouvées. Ces valeurs sont signées et indiquent si la corde part vers l'avant (>0) ou vers l'arrière de la position courant (<0). L'arbre produit est nécessairement planaire (propriété héritée de SEGMENT). Attention ! La fonction fixe N = n.

    STRATÉGIE. On part du centre vers le point 0 du centre, puis chacun des deux robots réveillent respectivement les k = ceil((n-1)/2) robots précédents et les n-1-k robots suivants en appliquant segment_c(k,n-1-k,n). En effet, à cause de la symétrie des points, segment_c(a,b) = segment_c(b,a), et donc on peut se limiter à k<n/2. La complexité est celle de segment_c(k,n-k) qui est maximum lorsque N = n et k ~ n/3, ce qui fait O(n^5).

    Variable globale modifiée: N, UPPER_BOUND, PROCESS_TIME.
    """
    global N, UPPER_BOUND, PROCESS_TIME
    if n == None: n = N # valeur par défaut ne marche pas sinon
    N = n # fixe la variable globale pour chord()
    UPPER_BOUND = 4 # majorant sur segment_c()
    segment_c.cache_clear() # efface cache, sinon résultats incorrects
    chord.cache_clear() # efface cache, sinon résultats incorrects

    if n == 0: return 0, # pas d'arbre dans ce cas
    k = ceil((n-1)/2)  # ici k>=0, car n>0

    PROCESS_TIME = process_time()
    x,T = segment_c(k, n-1-k)
    PROCESS_TIME = process_time() - PROCESS_TIME

    return 1 + x, T

def convert_tree(r, T, n=None):
    """
    Renvoie un arbre enraciné T = [ r, T_1 ... T_k ] en fonction d'une racine r (indice dans [0,n[) et de l'arbre relatif renvoyé par SEGMENT_CM() ou SEGMENT_C(). Attention ! Les points du cercle sont numérotés dans [0,n[ ce qui veut dire que la source ne devra pas appartenir à cet intervalle et être ajoutée à la fin de POINTS. L'algorithme est linéaire en la taille de T.
    """
    if n == None: n = N

    if len(T) == 1: return [r] # la racine est c'est tout
    if len(T) == 2: return [r, [(r+T[0]+n)%n]] # la racine est son fils
    return [r, convert_tree((r+T[0]+n)%n,T[1],n), convert_tree((r+T[2]+n)%n,T[3],n)]

@lru_cache(maxsize=None)
def segment(k1,r,k2):
    """
    Donne le temps et l'arbre optimal pour que deux robots réveillés et placés en POINTS[r] réveillent les k1 (resp. k2) robots situés avant (resp. après) r, le réveil des robots devant être réalisé seulement en utilisant les robots du segment [r-k1,r[ (resp. ]r,r+k2]). En particulier, si k1=0 (resp. k2=0), les deux robots partent réveiller à deux le segment ]r,r+k2] (resp. [r-k1,r[).

    L'algorithme est de complexité O( k1^2 * k2^2 * |k1-k2| * N ) car: (1) le coût de la fonction sans les appels récursifs est comme segment_c(k1,k2) soit O( k1^2 * k2^2 * |k1-k2| ); et (2) grâce à la mémorisation, il n'y a que O( k1 * N * k2) appels différents.
    """
    if k1 == k2 == 0: return 0, [r] # on s'arrête dans ce cas
    n = len(POINTS)-1 # on fait tout modulo n (en excluant le dernier point qui est ROOT)

    # CAS 1: k1,k2>0. Il faut trouver la meilleur corde de chaque coté
    # (en parallèle), soit le meilleur t dans [1,k1], puis dans [1,k2]
    if k1 > 0 and k2 > 0:
        x1min = x2min = UPPER_BOUND # majorant sur la valeur retournée
        for t in range(1, k1+1): # meilleure corde avant r dans [1,k1]
            v = (r-t+n)%n # v=r-t (modulo n)
            x,T = segment(k1-t, v, t-1); x += dist(r,v)
            if x < x1min: x1min, T1min = x, T
        for t in range(1, k2+1): # meilleure corde après r dans [1,k2]
            v = (r+t)%n # v=r+t  (modulo n)
            x,T = segment(t-1, v, k2-t); x += dist(r,v)
            if x < x2min: x2min, T2min = x, T
        return max(x1min, x2min), [r, T1min, T2min] # le pire des deux branches

    # CAS 2: k1=0 ou k2=0. Les cordes t1,t2 partent toutes deux du même coté:
    # avant r si k2=0 ou après r si k1=0. Il faut donc réveiller les robots
    # à l'aide de deux robots et donc trouver 2 cordes t1<t2 tout et en
    # découpant l'intervalle ]t1,t2[

    # on s'arrange pour avoir k1=0 et k2>0
    s = +1 # signe: +1 si après, -1 si avant
    if k1 > 0: # donc k2=0
        k1,k2 = k2,k1 # échange k1 et k2
        s = -1

    # CAS 2.1: k2=1, un fils
    if k2 == 1:
        v = (r+s+n)%n
        return dist(r,v), [r, [v]]

    # CAS 2.2: k2>=2, deux fils
    m = UPPER_BOUND # majorant sur la valeur retournée
    for t1 in range(1, k2):  # t1=1..k2-1, NB: k2>=2
        v1 = (r+s*t1+n)%n # v1=r+t1 ou r-t1 (modulo n)
        c1 = dist(r,v1)
        for t2 in range(t1+1, k2+1):  # t2=t1+1..k2 (t2=t1 inutile si métrique)
            v2 = (r+s*t2+n)%n # v2=r+t2 ou r-t2 (modulo n)
            c2 = dist(r,v2)
            k = t2-t1-1 # il y a k>=0 robots à se partager entre t1 et t2
            for i in range(0, k+1): # on en prend i pour t1 et k-i pour t2
                x1,T1 = segment(t1-1, v1, i) if s>0 else segment(i, v1, t1-1)
                x2,T2 = segment(k-i, v2, k2-t2) if s>0 else segment(k2-t2, v2, k-i)
                x = max(c1+x1, c2+x2)
                if x < m: m,T1min,T2min = x,T1,T2 # on a trouvé mieux
    return m, [r, T1min, T2min]

def SEGMENT(P=None, d=None):
    """
    Renvoie le temps et l'arbre optimal de réveille d'une liste de points P depuis un robot réveillé et placé à la fin de P (donc placé en P[n] où n = |P|-1) de sorte que les points de chaque sous-arbre forment un ensemble consécutifs de P. On se base sur segment(k,v,n-1-k) pour un certain premier point v et entier k >= 0 qui sont à optimiser (un peu comme dans OPTIMAL). On suppose que P contient au moins un point, même si la fonction est définie à la main pour n = 0. La liste P n'est pas modifiée, mais ROOT est fixé à n. On suppose le cas métrique pour d.

    On peut aussi obtenir x,T = SEGMENT_C(n) avec:

        POINTS = generate_regular_polygon(n) + [(0,0)] # un cercle plus son centre
        x,T = SEGMENT()

    C'est plus lent et on obtient pas forcément le même arbre. Cela vient du fait que les appels récursifs ne sont pas les mêmes: segment_c(k1,k2) vs. segment(k1,s,k2). Du coup, pour k1 et k2 fixés, les arbres pour segment(k1,s,k2) et segment(k1,s',k2) pourraient être différents alors qu'avec segment_c(k1,k2) il sera bien évidemment exactement le même. La complexité de l'algorithme est O(n^2) fois celle de segment(n/3,r,n/3), soit O(n^8).

    Variables globales modifiées: ROOT, POINTS, DIST, UPPER_BOUND, PROCESS_TIME.
    """
    global ROOT, POINTS, DIST, UPPER_BOUND, PROCESS_TIME
    if P == None: P = POINTS # valeur par défaut ne marche pas sinon
    if d == None: d = DIST # valeur par défaut ne marche pas sinon
    n = len(P)-1 # nombre de points sans le dernier, qui doit être ROOT
    ROOT = n # fixe la variable globale
    POINTS = P # fixe la variable globale pour dist() et segment()
    DIST = d # fixe la variable globale pour dist() et segment()
    UPPER_BOUND = 100*eccentricity(ROOT,P,d) # 2*(n-1)*eccentricity(ROOT,P,d) # fixe la variable globale pour optimal() qui peut correspondre dans le pire des cas à la longueur de la plus mauvaise branche
    segment.cache_clear() # sinon résultats incorrects
    dist.cache_clear() # sinon résultats incorrects

    if n == 0: # arbre non défini dans ce cas
        PROCESS_TIME = 0 # temps de calcul nul dans ce cas
        return 0, None

    PROCESS_TIME = process_time()

    m = UPPER_BOUND # majorant
    for v in range(n): # pour tout premier point v=0..n-1, NB: v!=ROOT et n>=1
        y = dist(ROOT,v) # ROOT->v
        for k in range(n): # pour tout nombre de points avant v, k=0..n-1
            x,T = segment(k,v,n-1-k)
            x += y
            if x < m: # on a trouvé une paire (v,k) meilleure
                m,Tmin = x,T

    PROCESS_TIME = process_time() - PROCESS_TIME

    return m, [ROOT, Tmin]

def GREEDY_SEGMENT(P=None, d=None):
    """
    Applique SEGMENT sur un ensemble de points qui a été précédemment ordonné selon un chemin glouton (point de plus proche) en partant du dernier point de P. Ainsi, les n = |P|-1 premières positions de P sont réordonnées, et le point le plus proche de la source, P[n], est P[n-1]. Comme SEGMENT(), ROOT est fixé à n. La complexité ajoute O(n^2) à la complexité de SEGMENT.

    Variables globales modifiées: ROOT et UPPER_BOUND (via SEGMENT), POINTS, DIST, PROCESS_TIME.
    """
    global POINTS, DIST, PROCESS_TIME
    if P == None: P = POINTS # valeur par défaut ne marche pas sinon
    if d == None: d = DIST # valeur par défaut ne marche pas sinon
    n = len(P)-1 # nombre de points sans le dernier, qui doit être ROOT
    POINTS = P # fixe la variable globale pour dist() et segment()
    DIST = d # fixe la variable globale pour dist() et segment()

    # attention de ne pas utiliser dist() qui est caché car les points
    # sont potentiellement réordonnés et donc dist(i,j) peut varier.

    PROCESS_TIME = process_time()

    for i in range(n,0,-1): # pour tout i=n..1
        xmin = d(P[i],P[i-1]) # cherche le point P[j] le plus proche de P[i]
        for j in range(i-2,-1,-1): # pour tout j=i-2..0
            x = d(P[i],P[j])
            if x < xmin: xmin,P[i-1],P[j] = x,P[j],P[i-1] # mise à jour avec échange
    
    x,T = SEGMENT(P,d) # applique SEGMENT

    PROCESS_TIME = process_time() - PROCESS_TIME 
    return x,T


#############################
#
# ALGORITHME: OPTIMAL(r,P,d)
#
#############################

"""
Algorithme optimal de réveil d'un ensemble de points P quelconque, depuis un point r de P et pour une fonction de distance d quelconque (pas forcément métrique, pas d'inégalité triangulaire donc).

On va faire une analogie avec le voyageur de commerce (TSP). Le brute-force, de manière général, consiste à tester toutes les "sorties" possibles. Pour le TSP, c'est un ordre sur les n points, ce qui donne un algorithme brute-force en n!*poly(n) où poly(n) est pour les détails de l'implémentation. Pour Freeze-Tag (FT), la sortie est un arbre étiqueté par les n points, soit un algorithme brute-force en n! * 2^n * poly(n) ou à peu près.

Le célèbre algorithme d'Held-Karp pour le TSP casse la barrière du n! en affirmant qu'on a pas forcément besoin de balayer toutes les sorties possibles pour trouver la meilleure. C'est assez surprenant de prime abord, et leur algorithme est en 2^n * poly(n) ce qui a des conséquences pratiques puisque qu'on passe de n=12 à n=26 en moins de 30" (avec mon implémentation en C). Et bien, j'ai un algorithme exact pour FT en c^n * poly(n) qui comme Held-Karp fonctionne pour n'importe quelle fonction de distance. Ici c = 3. Notez que Held-Karp est le meilleur algorithme connue pour le TSP où d est non-métrique. C'est facile à implémenter, donc je vais le faire. Je pense que pour n ~ 10 ça devrait le faire. (PS: Effectivement cela passe et même on peut aller jusqu'à 17 points car la complexité, le poly(n) et c, sont plus faibles que ce que je pensais au départ.) Notons qu'on pourra tester la métrique que l'on veut si cela nous intéresse.

D'ailleurs, en conservant l'analogie avec TSP, je me demande si on ne peut pas montrer le même phénomène de non-localité, c'est-à-dire le fait de déplacer un peu un point et que la branche la plus profonde change complètement. Car c'est cela qui rend difficile le TSP (changement complet de la tournée en déplaçant juste un point). Bon, à voir.

Une propriété qu'on va utiliser elle la suivante: 

PROPOSITION (Un fils). Dans le cas métrique, tout noeud d'un arbre optimal de degré deux, doit être connecté à une feuille. (En fait, il faudrait dire qu'il existe toujours un arbre optimal où les noeuds de degré deux sont connectés à des feuilles.)

PREUVE. Sinon, son père pourrait informer plus rapidement un de ses petits fils. Pour le 2e point, soit h la hauteur d'un arbre optimal.
QED

Il est faux de dire que l'arbre est hauteur h = ceil{log_2(n)}. En fait, on peut quasiment avoir n'importe quelle hauteur h > log(n) désirée (l'exemple 1 des slides de la réunion TEMPORAL montre d'ailleurs un exemple avec n=8 points avec la source et la plus longueur branche comprenant 4 arêtes ...). Une fois arrivé à deux robots, il est tout à fait possible d'avoir un sous-arbre de petite hauteur mais avec de longues branches et l'autre sous-arbre avec une grande profondeur mais des points relativement très proches les uns des autres.

PROPOSITION (Un fils, bis). Soient u-v-w trois sommets d'une branche d'arbre optimal tel que w est une feuille de père v, lui même de père u, et avec v de degré deux. Alors, si d() est symétrique (en particulier dans le cas métrique), d(u,v) <= d(u,w). (En fait, il faudrait dire qu'il existe toujours un arbre optimal où d(u,v) <= d(u,w).)

PREUVE. Dans l'arbre optimal, d(u,v) + d(v,w) <= d(u,w) + d(w,v), car sinon on pourrait utiliser la branche u-w-v au lieu de u-v-w. (NB: Ce n'est forcément vrai si v a un troisième voisin.) En particulier, comme d(v,w) = d(w,v), cela implique que d(u,v) <= d(u,w).
QED

EN fait cette dernière proposition n'est pas forcément très intéressante en pratique car elle est de fait réalisée par la fonction optimal(r,A,b). C'est un peu comme si on prenait 4 points et qu'on exprimait le résultat en fonction de minimum sur des ensembles (ou partitions) plus petite.

PROPOSITION (Non croisements). Dans le cas métrique, il existe toujours un arbre optimal sans que deux robots ne puissent se croiser (au même moment et même endroit).

PREUVE [À FINIR]. On part d'un arbre optimal avec des segments de droites. Si deux chemins se croisent au même moment, disons au point w, c'est que la distance à la source est la même. On considère les deux arêtes uv et xy de l'arbre, qui sont des segments de droites, qui s'intersectent en w. On peut modifier l'arbre en échangeant uv avec uy et xy avec xv. Il faut montrer que la plus longue des deux branches a diminuée ... [À FINIR] La somme totale des longueurs des arêtes a strictement diminuée. Il suit qu'en répétant la procédure, on obtient un arbre sans aucun croisement. La procédure de décroisement, même si elle converge, pourrait ne pas être polynomiale.
QED

Plus en détails. On se base sur optimal(r,A,b) qui renvoie l'arbre optimal de réveille du sous-ensemble de points A à partir d'1 ou 2 robots (dépendant de b) réveillés et placés en r.

Pour tout point r et ensemble A ne contenant pas r:
    optimal(r,A,1) = 0 si |A| = 0
    optimal(r,A,1) = min_{u∊A} { dist(r,u) + optimal(u,A\{u},2)} si |A| > 0
    optimal(r,A,2) = min_{A1,A2} max{ optimal(r,A1,1), optimal(r,A2,1) }, où {A1,A2} est une partition de A (si dist() est une métrique on peut forcer à ce que |A1|=0 => |A2|=1 (ou le contraire) de sorte que les sommets de degré deux (donc avec 1 fils) de l'arbre mène nécessairement à une feuille. Il faut alors distinguer le cas |A|=0, |A|=1 et |A| >= 2.)

"""

@lru_cache(maxsize=None)
def optimal(r,A,b):
    """
    Renvoie le temps et l'arbre optimal de réveille du sous-ensemble A (sous-ensemble d'indices de POINTS) à partir de robots réveillés et positionnés en r (indice de POINTS). Si b=True, il n'y a qu'un seul robot réveillé (et r possédera un seul fils dans l'arbre) et sinon deux sont réveillés (et r possédera deux fils). Le sous-ensemble A ne doit pas contenir r. Il peut être vide si b=True mais il doit contenir au moins un élément si b=False. On se place dans le cas métrique, c'est-à-dire que DIST vérifie l'inégalité triangulaire. Attention ! A est ici un tuple, pas une liste, de façon à être "hashable" ce qui est important pour @lru_cache().

    STRATÉGIE

        CAS 1 (b=True): il faut choisir la meilleure cible v dans A, puis réveiller tous les points de A\{v} avec deux robots placés v en appliquant optimal(v,A\{v},False) et ainsi créer un arbre de racine r avec un fils, v et son sous-arbre.

        CAS 2 (b=False): il faut couper A en deux sous-ensembles, A1 et A2, chacun avec au moins un élément, et appliquer la stratégie optimal(r,A1,True) et optimal(r,A2,True) et ainsi créer un arbre de racine r avec deux fils. Il faut bien sûr tester toutes les partitions {A1,A2} et prendre le meilleur arbre. Dans le cas non-métrique, il faudrait tester également le cas où un seul robot se déplace (ou disons le cas où les deux robots se déplacent le long de la même arête). C'est inutile dans le cas métrique.

    Notons a = |A| et p = |POINTS|. La complexité est alors O( a*p * 2^a * binom(p,a) ) car: (1) le coût de la fonction sans les appels récursifs est O( 2^a * a ) à cause du CAS 2; et (2) grâce à la mémorisation, le nombre d'appels différents est O( p * binom(p,a) ) car les ensembles B (CAS 1) puis A1 et A2 (CAS 2) sur lesquels on rappelle la fonction sont des sous-ensembles de A. On va maintenant donner une borne seulement en fonction de p, ce qui servira pour l'analyse de complexité de la fonction OPTIMAL(). On va faire mieux qu'un naïf O( p^2 * 4^p ) en majorant a <= p et binom(p,a) <= 2^p.
    
    Posons x = a/p. Il est bien connu que, pour tout ratio constant x = 0..1, binom(p,px) ~ h(x)^p / √(2𝜋x(1-x)p) où h(x) = (1/x)^x * (1/(1-x))^(1-x). On va se concentrer sur le terme le plus important, la base de l'exposant p. Du coup le terme 2^a * binom(p,a) devient 2^px * binom(p,px) ≃ (2^x * h(x))^p. Il suffit maintenant de "voir" quand est-ce que le terme 2^x * h(x) est maximum. De manière analytique Maple dit que maximize(2^x*(1/x)^x*(1/(1-x))^(1-x), x=0..1, location) vaut 3 réalisé pour x = 2/3. Un autre argument, tout aussi valable permet de montrer que c'est effectivement que 2^a * binom(p,a) <= 3^p. En effet, le terme 2^a code un sous-ensemble A' de A (en fait deux, A1 et A2 qui sont complémentaires dans A) et que le terme binom(p,a) code le sous-ensemble A de POINTS. Bref, le produit 2^a * binom(p,a) code un couple (A',A). Combien y'a-t'il de tels couples ? Autant que de mots M sur {1,2,3} avec p lettres, avec le codage suivant (du coup il y en a 3^p):
    
        M[i] = 1 si l'élément i de POINTS appartient à A'
        M[i] = 2 si l'élément i de POINTS appartient à A\A'
        M[i] = 3 si l'élément i de POINTS n'appartient pas à A

    Du coup, il y en a bien au plus 3^p. Bref, on peut majorer la complexité O( ap * 2^a * binom(p,a) ) par O( p^{3/2} * 3^p ) (ne pas oublier le 1/√p dans binom(p,px)). Notons qu'il manque sans doute un terme en log(n) pour la mémorisation, bien qu'il soit sans doute possible de l'implémenter en temps O(1), au moins de manière amortie.

    NB. Il faut être vigilant sur la gestion des listes et veiller à ne pas trop en générer. On pourrait coder les sous-ensembles par des entiers, peut-être pour gagner en vitesse et utiliser plus efficacement le cache. En tout cas, la présence de tuple(A)/list(A) dans les appels récursifs sont nécessaires pour éviter l'erreur "TypeError: unhashable type: 'list'" (on ne peut pas mettre dans le cache les appels de fonctions qui contiennent des listes, mais c'est ok pour les tuples).

    AMELIORATIONS ?
    
    On pourrait inclure une borne inférieure de façon à couper des branches de calculs inutiles. Cependant à cause de la mémorisation, il n'est pas sûr que cela améliore le temps car on va de toutes façons tout calculer. L'espoir n'est pas de gagner sur le nombre valeurs (r,A,b) possibles (qui va en fait augmenter), mais sur la complexité de la fonction, notamment le CAS 2, sur le terme 2^{|A|}. On peut espérer couper avant de tout faire.

    Pour le cas métrique, comme borne inf, on pourrait se baser sur LB(r,A) = h(r) + ecc(r,A), la hauteur déjà calculée pour atteindre r plus la plus grande distance restant à faire pour réveiller A depuis r. Si LB(r,A) est supérieure au meilleur arbre déjà calculé, cela n'est pas la peine de continuer et de partitionner A, et gagner ainsi sur le terme en 2^{|A|}. En ajoutant un paramètre h, pour indiquer la profondeur atteinte par r, la mémorisation marchera. Il y aura une perte en complexité qu'il faudra multiplier par le nombre de valeur possible pour la borne inf, disons NB_DIGIT. Donc pour espérer gagner du temps, il faut impérativement avoir NB_DIGIT < |POINTS|/3 (soit la taille critique pour A). (NB_DIGIT = 3 pourrait être un bon point de départ). En fixant h = -1 (une constante), on pourrait permettre à la fonction de ne pas tenir compte de la borne inf, et ainsi retrouver exactement les mêmes complexité et performances que la version optimal(r,A,b) originale.
    
    Cependant, ce qui n'est pas clair est de savoir comment récupérer une borne sup sur l'arbre le meilleur trouvé ? Naïvement, le meilleur arbre déjà calculé pourrait être déterminé à chaque fois qu'on atteint un cas terminal, soit lorsque |A| = 0 ou 1. Mais ce n'est pas vrai, car dans les cas terminaux on se sait pas si en parallèle il y a, il y aura, ou il n'y aura pas de branche plus longue. Une possibilité est de fixer UPPER_BOUND = SEGMENT()[0]. Une autre borne sup possible plus rapide à calculer est somme_{i=0}^{n-2} dist((r+i+r)%n,(r+i+1)%n) en O(n), ou encore un chemin glouton (style point le plus proche) en O(n^2). À voir.
    """

    n = len(A)
    if n == 1: return dist(r,A[0]), [r, list(A)] # c'est fini: r a un fils qui est une feuille
    xmin = UPPER_BOUND # majorant sur le résultat

    if b:
        ############################
        # CAS 1: un robot réveillé #
        ############################

        if n == 0: return 0, [r] # c'est fini: r est une feuille
    
        # cherche le meilleur point v de A à réveiller, NB: ici |A|>=2
        C = list(A) # évite de faire list(A) dans la boucle
        for v in A: # pour chaque v possible de A
            B = C[:] # B = copie de A
            B.remove(v) # B = B\{v} = A\{v}, NB: pas possible de faire remove si tuple ...
            x,T = optimal(v,tuple(B),False) # calcul de la solution pour (v,B)
            x += dist(r,v) # ajoute le temps r->v
            if x < xmin: # on a trouvé mieux en commençant par v
                xmin,Tmin = x,T # on garde la meilleure solution

        return xmin, [r, Tmin] # arbre avec un fils

    #########################
    # Deux robots réveillés #
    #########################

    # Il faut couper A en deux sous-ensembles, A1 et A2, chacun avec au moins 1 point ce qui est possible car ici |A|>=2. Pour cela on représente un sous-ensemble par un mot binaire w sur |A| bits. Les bits à 1 de w sont les éléments de A1, les bits 0 ceux de A2. On évite w=000...00 et w=111...11 de façon à garantir |A1|,|A2|>=1. Comme {A1,A2} forment une partition de A, on peut supposer que pour A1 on va de w=000...01 à 011..11. Enfin, on va s'arranger pour que |A1|>=|A2| afin de couper plus vite car a priori c'est pour les grands ensembles que l'arbre est le moins profond.

    for w in range(1,2**(n-1)): # w = 000..01 à 011..11

        # construit A1 et A2 en fonction de w
        A1 = []; A2 = [] # surtout ne pas mettre A1=A2=[] ...
        for i in range(n): 
            if w & 1: # teste le dernier bit de w
                A1 += [A[i]]
            else: A2 += [A[i]]
            w >>= 1 # enlève le dernier bit à w
            # NB: cela ne modifie pas le w de la boucle "for w in range(...)"

        if len(A1)<len(A2): A1,A2 = A2,A1 # ici |A1|>=|A2|

        x1,T1 = optimal(r,tuple(A1),True) # 1 robot réveille A1
        if x1 >= xmin: continue # pas la peine de continuer
        x2,T2 = optimal(r,tuple(A2),True) # 1 robot réveille A2
        x = max(x1,x2) # la plus grande des deux branches
        if x < xmin: # on a trouvé un meilleur arbre
            xmin,T1min,T2min = x,T1,T2

    # Ici T1min et T2min ont la même racine r. Il faut les fusionner.
    # Ex: si T1min=[r,T1'] et T2min=[r,T2'] (un fils chacun)
    # alors -> [r, T1', T2'] (deux fils pour r)

    return xmin, [r, T1min[1], T2min[1]]

def OPTIMAL(r=None, P=None, d=None):
    """
    Renvoie le temps et l'arbre optimal de réveille pour un ensemble de points P depuis un robot réveillé r qui doit être un indice de P. Il faut donc |P| >= 1 et r dans [0,|P|[. La liste P n'est pas modifiée. On se place dans le cas métrique, c'est-à-dire que DIST vérifie l'inégalité triangulaire. 

    La complexité de l'algorithme est majorée par l'appel à optimal(r,A,b), soit O( n^{3/2} * 3^n ) avec n = |POINTS|. En pratique, pour n = 15+1 points, cela prend 1'35" et pour n = 16+1 points, 5'30". Notons que 17! = 355,687,428,096,000 ~ 4.1 jours). Donc cela confirme que l'algorithme est significativement plus rapide que n!.

    Variables globales modifiées: ROOT, POINTS, DIST, UPPER_BOUND, PROCESS_TIME.
    """
    global ROOT, POINTS, DIST, UPPER_BOUND, PROCESS_TIME
    if r == None: r = ROOT # valeur par défaut ne marche pas sinon
    if P == None: P = POINTS # valeur par défaut ne marche pas sinon
    if d == None: d = DIST # valeur par défaut ne marche pas sinon
    ROOT = r # fixe la variable globale
    POINTS = P # fixe la variable globale pour optimal()
    DIST = d # fixe la variable globale pour optimal()
    n = len(P)
    UPPER_BOUND = 100*eccentricity(ROOT,P,d) # 2*(n-1)*eccentricity(ROOT,P,d) # fixe la variable globale pour optimal() qui peut correspondre dans le pire des cas à la longueur de la plus mauvaise branche
    optimal.cache_clear() # sinon résultats incorrects
    dist.cache_clear() # sinon résultats incorrects
    
    PROCESS_TIME = process_time()

    if n == 1: return 0, [r] # aucun point à réveiller, le seul point r l'est déjà
    A = list(range(n)) # A = [0,|P|[ = indices de P
    del A[r] # A = [0,r[ u ]r,|P|[ = indices de P sauf r
    x,T = optimal(r,tuple(A),True) # ici r doit réveiller A

    PROCESS_TIME = process_time() - PROCESS_TIME

    return x,T


######################################################################
#
# Affiche les premières valeurs pour n points réguliers sur un cercle
#
######################################################################

DIST = dist_L2 # fonction de distance

for n in range(10):
    print(f"T_segment_cm({n}) = {SEGMENT_CM(n)[0]:.3f}", sep="", end="\t")
    print(f"T_segment_c({n}) = {SEGMENT_C(n)[0]:.3f}", sep="", end="\t")
    POINTS = generate_regular_polygon(n) + [(0,0)] # un cercle plus son centre
    print(f"T_segment({n}) = {SEGMENT()[0]:.3f}", sep="", end="\t")
    print(f"T_optimal({n}) = {OPTIMAL(n)[0]:.3f}", sep="", end="\t")
    if n%2 == 0 and n>0:
        x=1+2*(sin(pi/n)+cos(pi/n))
        print(f"theo({n}) = {x:.3f}", sep="", end="\t")
    print()

########################################
# Quelques résultats déjà calculés pour
# n points uniformes sur le cercle
########################################

# Pour n=4,6,8,10,12, T_optimal(n) = 1+2(sin(pi/n)+cos(pi/n))
# Ce n'est plus vrai pour n=14,16
#
# T_segment_cm(0) = 0.000     T_segment_c(0) = 0.000     T_segment(0) = 0.000     T_optimal(0) = 0.000
# T_segment_cm(1) = 1.000     T_segment_c(1) = 1.000     T_segment(1) = 1.000     T_optimal(1) = 1.000
# T_segment_cm(2) = 3.000     T_segment_c(2) = 3.000     T_segment(2) = 3.000     T_optimal(2) = 3.000
# T_segment_cm(3) = 2.732     T_segment_c(3) = 2.732     T_segment(3) = 2.732     T_optimal(3) = 2.732
# T_segment_cm(4) = 3.828     T_segment_c(4) = 3.828     T_segment(4) = 3.828     T_optimal(4) = 3.828
# T_segment_cm(5) = 3.351     T_segment_c(5) = 3.351     T_segment(5) = 3.351     T_optimal(5) = 3.351
# T_segment_cm(6) = 3.732     T_segment_c(6) = 3.732     T_segment(6) = 3.732     T_optimal(6) = 3.732
# T_segment_cm(7) = 3.431     T_segment_c(7) = 3.431     T_segment(7) = 3.431     T_optimal(7) = 3.431
# T_segment_cm(8) = 3.613     T_segment_c(8) = 3.613     T_segment(8) = 3.613     T_optimal(8) = 3.613
# T_segment_cm(9) = 3.416     T_segment_c(9) = 3.416     T_segment(9) = 3.416     T_optimal(9) = 3.416
# T_segment_cm(10) = 3.520    T_segment_c(10) = 3.520    T_segment(10) = 3.520    T_optimal(10) = 3.520
# T_segment_cm(11) = 3.383    T_segment_c(11) = 3.383    T_segment(11) = 3.383    T_optimal(11) = 3.383
# T_segment_cm(12) = 3.449    T_segment_c(12) = 3.449    T_segment(12) = 3.449    T_optimal(12) = 3.449
# T_segment_cm(13) = 3.349    T_segment_c(13) = 3.349    T_segment(13) = 3.349    T_optimal(13) = 3.349
# T_segment_cm(14) = 3.454    T_segment_c(14) = 3.454    T_segment(14) = 3.454    T_optimal(14) = 3.454
# T_segment_cm(15) = 3.318    T_segment_c(15) = 3.318    T_segment(15) = 3.318    T_optimal(15) = 3.318
# T_segment_cm(16) = 3.443    T_segment_c(16) = 3.443    T_segment(16) = 3.443    T_optimal(16) = 3.443
#
# T_segment_cm(40) = 3.289    T_segment_c(40) = 3.289    T_segment(40) = 3.289
# T_segment_cm(41) = 3.270    T_segment_c(41) = 3.270    T_segment(41) = 3.270
# T_segment_cm(42) = 3.277    T_segment_c(42) = 3.277    T_segment(42) = 3.277
# T_segment_cm(43) = 3.259    T_segment_c(43) = 3.259    T_segment(43) = 3.259
# T_segment_cm(44) = 3.265    T_segment_c(44) = 3.265    T_segment(44) = 3.265
# T_segment_cm(45) = 3.249    T_segment_c(45) = 3.249    T_segment(45) = 3.249
# T_segment_cm(46) = 3.255    T_segment_c(46) = 3.254    T_segment(46) = 3.254
# T_segment_cm(47) = 3.239    T_segment_c(47) = 3.239    T_segment(47) = 3.239
# T_segment_cm(48) = 3.255    T_segment_c(48) = 3.245    T_segment(48) = 3.245 <- SEGMENT devient meilleur
# T_segment_cm(49) = 3.231    T_segment_c(49) = 3.231    T_segment(49) = 3.231
# T_segment_cm(50) = 3.255    T_segment_c(50) = 3.235    T_segment(50) = 3.235
# T_segment_cm(51) = 3.225    T_segment_c(51) = 3.223    T_segment(51) = 3.223
# T_segment_cm(52) = 3.254    T_segment_c(52) = 3.227    T_segment(52) = 3.227
# T_segment_cm(53) = 3.225    T_segment_c(53) = 3.215    T_segment(53) = 3.215
# T_segment_cm(54) = 3.252    T_segment_c(54) = 3.219    T_segment(54) = 3.219
# T_segment_cm(55) = 3.226    T_segment_c(55) = 3.208    T_segment(55) = 3.208
# T_segment_cm(56) = 3.254    T_segment_c(56) = 3.212    T_segment(56) = 3.212
# T_segment_cm(57) = 3.225    T_segment_c(57) = 3.201    T_segment(57) = 3.201
# T_segment_cm(58) = 3.252    T_segment_c(58) = 3.205    T_segment(58) = 3.205
# T_segment_cm(59) = 3.228    T_segment_c(59) = 3.195    T_segment(59) = 3.195
# T_segment_cm(60) = 3.246    T_segment_c(60) = 3.201    T_segment(60) = 3.201
# T_segment_cm(61) = 3.229    T_segment_c(61) = 3.189    T_segment(61) = 3.189
# T_segment_cm(62) = 3.240    T_segment_c(62) = 3.204    T_segment(62) = 3.204 <- anomalie wrt n=60
# T_segment_cm(63) = 3.224    T_segment_c(63) = 3.184    T_segment(63) = 3.184
# T_segment_cm(64) = 3.234    T_segment_c(64) = 3.206    T_segment(64) = 3.206 <- anomalie wrt n=62,60
# T_segment_cm(65) = 3.220    T_segment_c(65) = 3.182    T_segment(65) = 3.182
# T_segment_cm(66) = 3.229    T_segment_c(66) = 3.204    T_segment(66) = 3.204 <- anomalie wrt n=60
# T_segment_cm(67) = 3.215    T_segment_c(67) = 3.185    T_segment(67) = 3.185 <- anomalie wrt n=65
# T_segment_cm(68) = 3.224    T_segment_c(68) = 3.201
# T_segment_cm(69) = 3.211    T_segment_c(69) = 3.186
# T_segment_cm(70) = 3.219    T_segment_c(70) = 3.197
# T_segment_cm(71) = 3.206    T_segment_c(71) = 3.183
# T_segment_cm(72) = 3.214    T_segment_c(72) = 3.194
# T_segment_cm(73) = 3.202    T_segment_c(73) = 3.180
# T_segment_cm(74) = 3.212    T_segment_c(74) = 3.190
# T_segment_cm(75) = 3.198    T_segment_c(75) = 3.178
# T_segment_cm(76) = 3.214    T_segment_c(76) = 3.187
# T_segment_cm(77) = 3.195    T_segment_c(77) = 3.175
# T_segment_cm(78) = 3.215    T_segment_c(78) = 3.183
# T_segment_cm(79) = 3.196    T_segment_c(79) = 3.172
# T_segment_cm(80) = 3.216    T_segment_c(80) = 3.180
# T_segment_cm(81) = 3.197    T_segment_c(81) = 3.169
# T_segment_cm(82) = 3.213    T_segment_c(82) = 3.177
# T_segment_cm(83) = 3.199    T_segment_c(83) = 3.167
# T_segment_cm(84) = 3.210    T_segment_c(84) = 3.174
# T_segment_cm(85) = 3.198    T_segment_c(85) = 3.164
# T_segment_cm(86) = 3.207    T_segment_c(86) = 3.171
# T_segment_cm(87) = 3.195    T_segment_c(87) = 3.162
# T_segment_cm(88) = 3.203    T_segment_c(88) = 3.168
# T_segment_cm(89) = 3.193    T_segment_c(89) = 3.159
# T_segment_cm(90) = 3.198    T_segment_c(90) = 3.166
# T_segment_cm(91) = 3.191    T_segment_c(91) = 3.157
# T_segment_cm(92) = 3.194    T_segment_c(92) = 3.163
# T_segment_cm(93) = 3.188    T_segment_c(93) = 3.155
# T_segment_cm(94) = 3.190    T_segment_c(94) = 3.160
# T_segment_cm(95) = 3.185    T_segment_c(95) = 3.152
# T_segment_cm(96) = 3.187    T_segment_c(96) = 3.158
# T_segment_cm(97) = 3.181    T_segment_c(97) = 3.150
# T_segment_cm(98) = 3.183    T_segment_c(98) = 3.155
# T_segment_cm(99) = 3.178    T_segment_c(99) = 3.148
# ...
# T_segment_cm(120) = 3.164   T_segment_c(120) = 3.141
# T_segment_cm(121) = 3.155   T_segment_c(121) = 3.134
# T_segment_cm(122) = 3.163   T_segment_c(122) = 3.140
# T_segment_cm(123) = 3.154   T_segment_c(123) = 3.133
# T_segment_cm(124) = 3.162   T_segment_c(124) = 3.139
# T_segment_cm(125) = 3.153   T_segment_c(125) = 3.132
# T_segment_cm(126) = 3.160   T_segment_c(126) = 3.138
# T_segment_cm(127) = 3.152   T_segment_c(127) = 3.131
# T_segment_cm(128) = 3.159   T_segment_c(128) = 3.136
# T_segment_cm(129) = 3.152   T_segment_c(129) = 3.130
# T_segment_cm(130) = 3.159   T_segment_c(130) = 3.135
# T_segment_cm(131) = 3.151   T_segment_c(131) = 3.129
# T_segment_cm(132) = 3.159   T_segment_c(132) = 3.134
# ...
# T_segment_cm(255) = 3.107   T_segment_c(255) = 3.084
# T_segment_cm(256) = 3.109   T_segment_c(256) = 3.086
# T_segment_cm(257) = 3.106   T_segment_c(257) = 3.084
# T_segment_cm(300) = 3.098   T_segment_c(300) = 3.075
# T_segment_cm(400) = 3.081   T_segment_c(400) = 3.062
# T_segment_cm(500) = 3.072   T_segment_c(500) = 3.055
# ...
# T_segment_cm(20000) = 3.007

#######
# MISC
#######
def angle(A,B):
    """
    Mesure de l'arc orienté de A à B. Le résultat est un angle de [0,2𝜋[.
    """
    c = 2*pi
    a = (atan2(*A)+c)%c # dans [0,2𝜋[ car atan2() dans [-𝜋,+𝜋]
    b = (atan2(*B)+c)%c # dans [0,2𝜋[ car atan2() dans [-𝜋,+𝜋]
    return (b-a+c)%c

def best_balanced(P,a):
    """
    Renvoie le triplet (i,k,b) tel que arc(P[i-1],P[i+1]) <= a et l'axe entre (0,0) et P[i] équilibre le plus le nombre de points des deux moitiés du cercle délimité cet axe, P[i] étant exclu. L'entier k est ce déséquilibre et b = arc(P[i-1],P[i+1]), angle non orienté donc dans [0,𝜋[. Il est nécessaire que P soit convexe et ordonné suivant son bord, que l'origine soit à l'intérieur et que a <= 4𝜋/|P|.
    """
    n = len(P)
    kmin = n+1 # valeur de k à renvoyer
    for i in range(n):

        # angle pas assez petit
        b = angle(P[(i-1+n)%n],P[(i+1+n)%n])
        if min(b,2*pi-b) > a: continue

        # compte les points dans chaque moitié par rapport à P[i]
        n1 = n2 = 0
        for j in range(n):
            if 0 < angle(P[i],P[j]) <= pi: n1 += 1
            else: n2 += 1

        k = abs(n1-n2) # équilibre du point P[i]
        if k < kmin: imin,kmin,bmax = i,k,b # on a trouvé mieux

    return imin, kmin, bmax

def test_best_balanced(n,r):
    """
    Teste l'hypothèse selon laquelle il existe toujours un point P[i] des n points de P avec arc(P[i-1],P[i+1]) <= 4pi/n et un déséquilibre d'au plus une certaine borne K. On répète r fois le tirage de n listes aléatoires. Pour le moment, je n'ai jamais trouvé K>3 (réalisé avec n=5). Mais cela vient peut-être que K=log(n) ou K=sqrt(n). Car, lorsque n grandit, les cas limites deviennent plus difficiles à trouver.
    """
    kmax = -1
    a = 4*pi/n
    for i in range(r):
        P = generate_von_mises(n,0) # cercle aléatoire
        P.sort(key=lambda A: atan2(*A)) # tri selon les angles
        i,k,b = best_balanced(P,a)
        if k > kmax:
            imax,kmax,bmax,Pmax = i,k,b,P[:]

    i,k,b,P = imax,kmax,bmax,Pmax
    P += [(0,0)] # pour visualiser le centre
    global ROOT
    ROOT = i
    draw_all(f'k = {k}\na = {b:.3f}\na_max = {a:.3f}',P=P)
    return

#test_best_balanced(5,10**6)
#quit()

######################
#
# Affiche les courbes
#
######################

X = range(10) # intervalle de valeurs pour n
mpl.rcParams['lines.marker'] = 'o'
mpl.rcParams['lines.markersize'] = 3

plt.grid(True) # affine une grille
plt.clf() # efface le dessin

Y = [SEGMENT_CM(n)[0] for n in X]
plt.plot(list(X), Y)

Y = [SEGMENT_C(n)[0] for n in X]
plt.plot(list(X), Y)

plt.legend(["segment_cm","segment_c"])
plt.show()


############################################
#
# Affiche des examples de points et d'arbre
#
########d####################################

# borne inférieure pour la norme L_p
LP_NORM = 1.5
DIST = dist_Lp

P = [(0,0)]
if LP_NORM<=2:
    P += [(1,0),(0,1),(-1,0),(0,-1)]
else:
    u = 1/2**(1/LP_NORM)
    P += [(u,u),(-u,u),(-u,-u),(u,-u)]

SEED = -1 # pour ne pas afficher la seed
PROCESS_TIME = 0 # temps écoulé en seconde pour affichage dans draw_all
u = 1+2**(1+max(1/LP_NORM,1-1/LP_NORM))
x,T = OPTIMAL(0,P)
draw_all(f'norm L_{LP_NORM}\nconjecture = {u:.3f}\nlower bound',x,T)

# contre-exemples en norme L_inf pour un carré 1x1
# 6+1 points et depth = 2.1
DIST = dist_Linf # fonction de distance par défaut, pour dist()
P = [
    (0,0),(0,1),(1,0),(1,1),
    (0,     .8),
    (1,     .4),
    (1,     .7)
]
SEED=-1 # pour ne pas afficher la seed
PROCESS_TIME = 0 # temps écoulé en seconde pour affichage dans draw_all
x,T = OPTIMAL(0,P)
draw_all('optimal',x,T)
quit()

# Exemple montrant que si n = 2^k alors l'arbre binaire n'est
# pas forcément l'optimal:
#
# init_seed(2074)
# N = 7 # nombre de points
# POINTS = generate_von_mises(N,0.5) # points uniforme sur le disque
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# PROCESS_TIME = 0
# SEED = -1
# draw_all()
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
# quit()

# Exemple montrant la différence entre SEGMENT (planaire)
# et OPTIMAL (pas forcément planaire) même pour des points
# en position convexe:
#
# init_seed(1993) # fixe SEED
# N = 7 # nombre de points
# POINTS = generate_convex(N) # points en position convexe
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# x,T = SEGMENT()
# draw_all('segment',x,T)
# x,T = OPTIMAL()
# draw_all('optimal',x,T)

# Exemple qui croise avec 4 points (hors 1ère arête) pour
# OPTIMAL en position convexe:
#
# init_seed(5687) # fixe SEED
# N = 4 # nombre de points
# POINTS = generate_convex(N) # points en position convexe
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
# quit()

# arbre de forme contrôlé ...
#
# init_seed() # fixe SEED
# x = 0.1
# h = 20
# POINTS = [(0,0), (0,h), (x,1), (h-1,1), (-x,3), (-h+3,3), (x,5), (h-5,5), (-x,7), (-h+7,7), (0,0)]
# N = len(POINTS)-1
# ROOT = N
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
# quit()

# 8 points en deux carrés de rayon différents
# r2 = 0.5, 0.8 ou 0.9 donnent des solutions assez différentes
# init_seed() # fixe SEED
# n = 4 # coté du polygone
# N = 2*n
# r1, r2 = 1.0, 0.5
# POINTS = []
# for i in range(n):
#     POINTS += [(r1*cos(i*2*pi/n),r1*sin(i*2*pi/n))]
#     POINTS += [(r2*cos(i*2*pi/n+pi/n),r2*sin(i*2*pi/n+pi/n))]
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
# quit()

# Exemple avec 3 croisements (hors 1ère arête) pour
# OPTIMAL en position convexe:
#
init_seed(3906) # fixe SEED
N = 11 # nombre de points
POINTS = generate_convex(N) # points en position convexe
normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
x,T = OPTIMAL()
print(T)
draw_all('convex pts')
draw_all('optimal',x,T)
# T = [11, [5, [6, [7, [8, [9], [10]], [4]], [3, [2]]], [0, [1]]]]
T = [11, [5, [6, [7, [8, [9], [10]], [4]], [0, [1]]], [3, [2]]]]
draw_all('???????',x,T)
quit()

# Exemple montrant la différence entre GREEDY_SEGMENT et OPTIMAL
# pour des points en position générale:
#
# init_seed(1620) # fixe SEED
# N = 11 # nombre de points
# POINTS = generate_von_mises(N,0.5) # points uniforme sur le disque
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# x,T = GREEDY_SEGMENT()
# draw_all('greedy_seg',x,T)
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
# quit()

# init_seed() # fixe SEED
# N = 17 # nombre de points
# x,T = SEGMENT_CM(N) # N = nb de points du cercle
# POINTS = generate_regular_polygon(N) # cercle uniforme
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# T = convert_tree(1,T,N) # converti en arbre enraciné au point 1 du cercle [0,N[
# T = [ROOT, [0, T]] # ajoute le point central et le point 0 du cercle à l'arbre
# draw_all('segment_cm',x,T)
# quit()

# init_seed() # fixe SEED
# N = 8 # nombre de points
# x,T = SEGMENT_C(N) # N = nb de points du cercle
# POINTS = generate_regular_polygon(N) # cercle uniforme
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# T = convert_tree(0,T,N) # converti en arbre enraciné au point 0 du cercle [0,N[
# T = [ROOT, T] # ajoute le point central à l'arbre
# draw_all('segment_c',x,T)
#quit()

# init_seed() # fixe SEED
# N = 8 # nombre de points
# POINTS = generate_regular_polygon(N) # cercle uniforme
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# SEED = - 1
# PROCESS_TIME = 0
# draw_all()
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
# x,T = SEGMENT()
# draw_all('segment',x,T)
# quit()

# init_seed() # fixe SEED
# N = 11 # nombre de points
# POINTS = generate_convex(N) # points en position convexe
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# x,T = SEGMENT()
# draw_all('segment',x,T)
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
# quit()

# init_seed() # fixe SEED
# N = 11 # nombre de points
# POINTS = generate_von_mises(N,0.5) # points uniforme sur le disque
# normalize_bc() # ajoute le centre ROOT = (0,0) en fin de liste
# x,T = GREEDY_SEGMENT()
# draw_all('greedy',x,T)
# x,T = OPTIMAL()
# draw_all('optimal',x,T)
#quit()

x = 3.570593145854709
T = [0, [4, [3, [8, [2]], [7]], [5, [1], [6]]]]
POINTS = [(0.0, 0.0), (-0.9985549564600698, 0.011827106584179929), (-0.006175724711713601, -0.9996790979426582), (0.9996797961518151, 0.018006435511381674), (0.7213105266396888, 0.6916888441126671), (-0.7102685631588145, 0.7017728500934621), (-0.040494833692801824, 0.9979330572465028), (-0.6851648746614944, -0.7272314994841055), (0.7196686298933903, -0.69431769612143)]
draw_all('toto',x,T)
quit()

# cherche le pire des cas pour N=8
n = 8 # T_segment_c(8)=3.613
ROOT = 0 # source pour POINTS

xmax = 0
for i in range(10000):
    # POINTS = generate_von_mises(n,0.1,2,20) # proche du polygone régulier
    POINTS = generate_von_mises(n,0)
    normalize_bc()
    x,T = OPTIMAL()
    if x>xmax:
        xmax,Tmax,Pmax = x,T,POINTS
        print(f"x = {x}")
        print(f"T = {T}")
        print(f"POINTS = {POINTS}")

# generate_von_mises(8,0,8,10): max sur 10,000
# x = 3.570593145854709
# T = [0, [4, [3, [8, [2]], [7]], [5, [1], [6]]]]
# POINTS = [(0.0, 0.0), (-0.9985549564600698, 0.011827106584179929), (-0.006175724711713601, -0.9996790979426582), (0.9996797961518151, 0.018006435511381674), (0.7213105266396888, 0.6916888441126671), (-0.7102685631588145, 0.7017728500934621), (-0.040494833692801824, 0.9979330572465028), (-0.6851648746614944, -0.7272314994841055), (0.7196686298933903, -0.69431769612143)]

# generate_von_mises(8,0.1): maximum sur 10,000
# (croisements de fous ...)
# x = 3.0498501436246337
# T = [0, [3, [1, [8, [2]], [7]], [5, [4], [6]]]]
# POINTS = [(0.0, 0.0), (-0.38313149525860823, -0.8944996655519788), (0.24967021557384517, 0.9683309266233882), (-0.1987667780636083, -0.8715771404052549), (-0.9335487858180874, 0.25552358830278954), (-0.6785219343019202, -0.6434257547307705), (0.8959422895728233, -0.26045916474825553), (0.7954899585093611, 0.5057387380199572), (0.2528665297861946, 0.9403684724901246)]

x,T,POINTS = xmax,Tmax,Pmax
draw_all('optimal',x,T)

"""
STRATÉGIE et ANALYSE du réveil de n robots placés de manière quelconque sur un cercle, avec un robot supplémentaire réveillé au centre.

Les robots sont en position p_0, p_1, ... p_{n-1} sur le cercle orientés dans le sens positif. On s'inspire des arbres calculés par SEGMENT_CM, qu'on appellera planaires monotones même si le terme peut être trompeur. Cela ne marche pour l'instant que si n >= 15, mais cela ne suppose pas le schéma existant en 10.1. La solution est planaire et monotone. Elle donne donc aussi une borne supérieure à la performance de SEGMENT (pour des points sur un cercle). La solution est calculable en temps O(n), une fois les points ordonnés sur le cercle. Sinon, je ne vois pas bien comment faire en moins que n*log(n), car sans ordre, pas facile de suivre les points sur le bord du cercle s'ils ne sont pas ordonnés. Ce qui prend O(n) c'est surtout le calcul de p_i (l'indice i). Pour p_k, je pense qu'on peut faire par dichotomie. Comme on va le voir, la preuve s'étend au cas général des points en position convexe, tout simplement car on peut se ramener au cas sur le cercle.

Pour deux points u,v du cercle, on note [u,v] l'arc orienté allant de u à v dans ce sens. On notera arc(u,v) la longueur de l'arc [u,v], et chord(u,v) la longueur de la corde qui vaut 2*sin(arc(u,v)/2).

----------------------------------------------------------------------------------------------------

ÉTAPE 1. Choisir un robot p_i tel que arc(p_{i-1},p_{i+1}) <= 4𝜋/n. C'est toujours possible, car la somme à n termes sum_{i=1}^n arc(p_{i-1},p_{i+1}) = 4𝜋 (indices modulo n) puisque chaque arc est compté deux fois. Un terme est donc en dessous de la moyenne.

    En fait, on voudrait plutôt un résultat qui disent qu'il existe toujours un i tel que le max{ arc(p_{i-1},p_i), arc(p_,p_{i+1}) } <= c𝜋/n avec c<4. Mais on ne peut pas dire mieux que c=4, car en prennant n=2k, dont k points doubles p_{2i} = p_{2i+1} (ou points infiniment proches), placés sur un k-gone régulier montrent que lorsqu'on prend trois points consécutifs, l'arc maximum sera 2𝜋/k = 4𝜋/n. Par contre, on verra à la fin de l'analyse (v3) que c > 1.952 (car sinon on peut montrer que la borne est vraie). Notons qu'on peut toujours supposer qu'il n'existe pas de points multiples puisque les points multiples ne peuvent pas produire une stratégie optimale plus lente que sans les points multiples dans la mesure où en temps nul on peut informer tous les robots placés au même point.

ÉTAPE 2. Réveiller p_i puis p_{i+1} et p_{i-1}. On va maintenant appliquer la même stratégie en parallèle sur chaque moitié de cercle. On ne décrit que la stratégie de p_{i+1} qu'on supposera la plus longue.

    C'est pour appliquer cet argument ("p_{i+1} est la branche la plus longue") qu'on a besoin des trois points spécifiques p_i, p_{i+1} et p_{i-1}. Si on avait pris les deux points les plus proches, disons p_i et p_{i+1} à distance <= 2𝜋/n, alors la stratégie en p_i et p_{i+1} ne serait pas la même, même en choisissant la meilleure des deux entre informer d'abord p_i puis p_{i+1} ou le contraire. Il faudrait donc considérer deux cas. Prendre trois points consécutifs proches semble donc plus simple à analyser. C'est bien dommage car si l'on remplace 4𝜋/n par 2𝜋/n dans la formule finale, alors on prouve la borne souhaitée pourtout n.

ÉTAPE 3. Soient B le point du cercle opposé à p_i et A le point de la moitié contenant p_{i+1} tel que l'arc [A,B] = t0 = 𝜋-2√2 < 0.3132. Soit p_k le dernier point (avec un robot) de l'arc [p_i,A].

    On augmentera plus tard t0 un peu pour le faire dépendre de n, plus précisément du nombre de robots dans [p_i,A]. Notons que A,B sont des points éventuellement sans robot. Le cas k = i+1 est possible.

ÉTAPE 4. Un robot en p_{i+1} se charge de réveiller tous les robots de [p_{i+1},p_k]. Il peut le faire en suivant le bord.

    On pourrait raffiner ici un peu en réappliquant la stratégie sur l'arc [p_{i+1},p_k], produisant alors un nouvel angle critique t1 et deux sous-étapes.

ÉTAPE 5. Si l'arc ]A,B] n'est pas vide, alors le second robot en p_{i+1} réveille p_{k+1} puis tous ceux de ]A,B]. Il peut le faire en suivant le bord.

    Comme dans le cas précédant, on pourrait raffiner en réappliquant la stratégie sur l'arc ]A,B] depuis p_{k+1}, produisant alors un nouvel angle critique t1 et potentiellement d'autres sous-étapes.

----------------------------------------------------------------------------------------------------

ANALYSE (v1): Clairement, la solution produit un arbre planaire et monotone.

Cas 1: ]A,B] est vide. Comme les branches sont monotones, on réveille tout point p_j de l'arc [p_i,p_k] en au plus 1 + arc(p_i,p_j) <= 1 + arc(p_i,p_k) <= 1 + 𝜋-t0 = 1 + 𝜋 - (𝜋-2√2) = 1+2√2, ce qui prouve la borne dans ce cas.

Cas 2: ]A,B] n'est pas vide. Il y a deux types de branches à analyser. Celles qui réveillent les points de l'arc [p_i,p_k], correspondent au cas 1, et donc vérifient la borne. Pour second type, la longueur est au plus

    1 + chord(p_i,p_{i+1}) + chord(p_{i+1},p_{k+1}) + arc(p_{k+1},B)        (1)

Par ailleurs on a:
. chord(p_i,p_{i+1}) <= arc(p_i,p_{i+1}) <= 4𝜋/n
. chord(p_{i+1},p_{k+1}) <= 2
. arc(p_{k+1},B) <= t0

C'est au plus 4𝜋/n + 3 + t0, ce qui est <= 1+2√2 dès que n > 4𝜋/(2√2-2-t0) = 4𝜋/(4√2-2-𝜋) ~ 24.4. La borne est donc démontrée dès que n >= 25.

ANALYSE (v2). On peut cependant être moins grossier dans les divers majorants ci-dessus. Si on pose x = arc(p_i,p_{i+1}) et y = arc(p_{k+1},B), la branche du second type vaut au plus

    1 + x + chord(p_{i+1},p_{k+1}) + y
    = 1 + x+y + 2sin((𝜋-(x+y))/2)
    = 1 + x+y + 2cos((x+y)/2), car sin(𝜋/2-u) = cos(u) pour tout u
    = 1 + z + 2cos(z/2) avec z=x+y <= 4𝜋/n + t0

En traçant la courbe de la fonction f: z -> 1+z+2cos(z/2) - (1+2√2), on s'aperçoit qu'elle est croissante et négative sur [0,z0) pour un certain z0 ~ 1.150093216. Comme z <= 4𝜋/n + t0, pour que f(z) soit négative et que la borne soit prouvée, il suffit donc de que 4𝜋/n + t0 <= z0, ce qui implique n >= 4𝜋/(z0-t0) ~ 15.1. La borne est donc démontrée dès que n >= 16.

    NB. On perd un peu sur x, qu'on pourrait remplacer par 2sin(x/2). Mais si on le fait, on perd le changement de variable avec z permettant de supprimer x et y. On perd aussi sur y qui devrait être plutôt quelque chose entre 2sin(y/2) et y, valeur dépendant du nombre de robots dans ]A,B]. Le nombre de points dans ]A,B] est au plus n-3 (manquent p_{i-1},p_i,p_{i+1}, k=i+1 étant possible), soit n-2 arcs. En appliquant les formules d'addition des sinus, on peut remplacer y par 2*(n-2)*sin(y/(2(n-2))) <= y. Du coup on pourrait remplacer z = x+y par 2sin(x/2) + 2(n-2)sin(z/(2(n-2))) <= 4(n-2)sin(z/(4(n-2))) <= z.

ANALYSE (v3). On peut encore gratter pour n "petit" comme suit. Supposons qu'on ait k+1 points u_0...u_k répartis sur un demi-cercle, délimitant k angles. (NB: il faut un demi-cercle car la corde vaut alors 2sin(t/2) où t est l'angle de la corde, ce qui est une fonction croissante lorsque t/2 est dans [0,𝜋/2], soit t dans [0,𝜋], soit le demi-cercle. Le fontion est concave pour tout t/2 dans [0,𝜋], mais on s'en fout.) Que vaut la somme des cordes, S = sum_{i=0..k-1} chord(u_i,u_{i+1}) ? Dans l'analyse, on majore S par L = arc(u_0,u_k), la somme des arcs. Mais l'intuition est que la somme des k cordes est maximisée quand les points sont uniformément répartis (c'est vrai au moins pour un nombre impaire de points, soit un nombre paire de cordes, cf. Proposition ci-après). Dit autrement, S = sum_{i=0..k-1} 2sin(arc(u_i,u_{i+1})/2) <= 2k sin(L/(2k)) ce qui est un peu moins que L surtout lorsque k est petit. Il faudrait donc injecter ceci dans l'analyse et jouer avec t0.

PROPOSITION (Somme des sinus). Soient {a_i} une suite de k angles positifs de somme au plus 2𝜋. Alors,

    sum_i sin(a_i)/k <= sin( sum_i a_i/k )

PREUVE (du cas pair). Soit k=2p un entier pair. La propriété est trivialement vraie pour p=0 (on a bien 0 <= 0). On coupe la somme en deux deux sommes de p termes, en posant x = sum_{i=1}^p a_i et y = sum_{i=p+1}^{2p} a_i. On a la formule bien connue d'addition des sinus: sin(x) + sin(y) = 2sin((x+y)/2) * cos((x-y)/2). Si x,y>=0 et 0 <= x+y <= 2𝜋, alors sin(x) + sin(y) <= 2sin((x+y)/2). En effet, dans ces conditions (x+y)/2 <= 𝜋, ce qui implique que sin((x+y)/2) ∈ [0,1]. Et bien sûr cos((x-y)/2) <= 1. En coupant en deux la somme, par induction et en appliquant le formule d'addition des sinus, il vient:

    sum_i sin(a_i) = sum_{i=1}^p sin(a_i) + sum_{i=p+1}^{2p} a_i         (on coupe en deux)
                   <= p*sin(x/p) + p*sin(y/p)                            (induction)
                   <= 2 (p*sin((x/p+y/p)/2)) = 2p * sin((x+y)/(2p))      (formule)
                   = k * sin( sum_i a_i/k )
QED

Je ne sais pas comment le prouver pour un nombre impaire de termes. Est-ce que cela découle de la concavité (car sin"(x) = cos'(x) = -sin(x) < 0 sur l'intervalle [0,𝜋[), et dire simplement que la moyenne d'une fonction concave est plus petite que la fonction de la moyenne ? Faut-il virer l'angle le plus grand ou du plus petit de la somme ? Des tests numériques avec Maple (plot3d) montrent que c'est vrai pour 3 angles. Mais a priori, cette formule est connue et peut être déduite de la formule Jensen et de la concavité de la fonction sinus ... (d'après https://les-mathematiques.net/vanilla/index.php?p=discussion/comment/1021853#Comment_1021853). La formule de Jensen fait le lien entre les zéros d'une fonction analytique (c'est-à-dire une fonction développable en série entière sur des points du plan, style exp(), cos(), sin() etc.) sur des points du disque unité et sur le cercle unité. Je préférerai une preuve avec des mots que je comprends.

L'inégalité de Jensen établit une relation entre la moyenne de valeurs d'une fonction convexe et la valeur de la moyenne des valeurs: https://fr.wikiversity.org/wiki/Fonctions_convexes/Applications_de_l%27inégalité_de_Jensen. On en déduit le résultat pour les fonctions concaves et donc pour sinus. Rappelons qu'une fonction f(x) est concave ssi -f(x) est convexe. Il suffit donc d'inverser les signes des inégalités.

On raffine maintenant la définition de t0 pour la faire dépendre de n. On redéfinit l'angle t0 tel que 2*(n-2)*sin((𝜋-t0)/(2*(n-2))) = 2√2. Cela revient à poser t0 = 𝜋 - 2*(n-2)*arcsin(√2/(n-2)). Notons que t0 > 𝜋-2√2 (en fait c'est sa limite), car pour tout x>0, arcsin(x) < x. On a même x/(1+x^2) < arcsin(x), mais on s'en fout.

Pour le cas 1, la longueur de branche est <= 1 + sum_{j=i..k-1} chord(p_j,p_{j+1}) = 1 + sum_j 2sin(arc(p_j,p_{j+1})/2). Le nombre de termes de cette somme est au plus k <= n-2, car au moins deux arcs entre p_k et p_i (incluant le point p_{i-1}) ne sont pas comptés. Cette somme vaut donc au plus 1 + 2*(n-2)*sin((𝜋-t0)/(2*(n-2))) = 1 + 2√2 par définition de t0 (si la formule de la somme des sinus est bien vraie pour le cas impaire).

NB. On doit pouvoir dire mieux sur le nombre de points k dans la 1ère branche (et gratter sur le "n-2" dans la définition de t0), car s'il n'y a qu'un seul point (p_{i-1}) dans la 2e partie, alors la longueur de cette branche serait <= dist(p_i,B) <= 2. Or on veut seulement prouver une longueur <= 2√2, ce qui laisse une peu de marge. En fait, la longueur de branche de la 2e partie est au plus la longueur du plus long périmètre maximum avec disons m arcs, soit 2m*sin(𝜋/(2m)). Il doit être <= 2√2. C'est vrai si m <= 2.

Pour le cas 2, on a déjà vu que pour que f(z) soit négative, et que la longueur de branche soit <= 1+2√2, il suffit que 4𝜋/n + t0 <= z0. Notons que z0 ne dépend pas de la définition de t0. Donc

    4𝜋/n + t0 <= z0  <=>  4𝜋/n + 𝜋 - 2*(n-2)*arcsin(√2/(n-2)) < 1.150093216

D'après le graphe de la fonction g: n -> 4𝜋/n + 𝜋 - 2*(n-2)*arcsin(√2/(n-2)), la plus petite valeur de n telle que g(n) < z0 est n ~ 14.91. La borne est donc prouvée dès que n >= 15. Notons que si on remplace "n-2" dans t0 par "n/2" par exemple, alors la borne est encore n >= 15 (n ~ 14.70). Et si dans la définition de z0 on remplace z+cos(z/2) par 2(n-2)sin(z/(2(n-2))) et n=13 par exemple, alors z0 est presque identique ~ 1.15.

De manière intéressante, on peut obtenir une borne inférieure sur l' arc contenant p_i, soit L = arc(p_{i-1},p_{i+1}) <= 4𝜋/n. En effet, si on remplace dans g(n) le terme 4𝜋/n par L, on s'aperçoit que la borne de 1+2√2 est vérifiée dès que L <= 1.952 𝜋/n. Plus précisément, on obtient que g(n) < z0 dès que n > 6.999903280, ce qui suffit car c'est déjà vrai pour n = 7. Du coup, on pourrait se concentrer sur le cas L > 1.952 𝜋/n. À voir si en revisitant la preuve cela aide. 

ANALYSE (v4 - TOTO).

Idée 1. On peut gratter sur la majoration chord(p_i,p_{i+1}) <= arc(p_i,p_i+1) <= 4𝜋/n, car le périmètre défini par les points de P est moins que 2𝜋, surtout lorsque n est petit. Donc, il faudrait prendre comme majorant plutôt 2P(n)/n où P(n) est le périmètre maximum d'un n-gone non-régulier inscrit dans un cercle de rayon unité. D'après la proposition du périmètre maximum, il vaut P(n) <= 2n*sin(𝜋/n). On peut donc remplacer partout chord(p_i,p_{i+1}) <= 4𝜋/n par chord(p_i,p_{i+1}) <= 4sin(𝜋/n) <= 4𝜋/n. Il faut refaire tous les calculs pour voir si on peut ainsi affiner les divers majorants.

Idée 2. On pourrait faire des bornes plus fines (que 1+2√2 pour n=4) on considérant le cas de l'exploration de non pas d'un demi-cercle [0,𝜋], mais d'un arc de cercle quelconque [0,t] avec t <= 𝜋. La constante 𝜋 devient alors t et la constante c = 2√2 pourrait devenir (peut-être) la somme maximum de deux cordes avec un point dans [0,t], soit c(t) = 2*(2sin(t/2)) = 4sin(t/2). Notons que pour t=𝜋 on a c(𝜋) = 2√2. Est-ce qu'un bon majorant serait 1 + c(4𝜋/n) = 1 + 4sin(2𝜋/n) ?

Idée 3. Pour la partie [p_{i+1},A] on peut utiliser un sous-arbre binaire de hauteur ceil(log(n-2)) en supposant un arbre montone optimal (cf. PROPOSITION "un fils"). Normalement, on gagne 1 sur le n minimum à partir duquel cela est vrai, donc passant à n >= 14. À confirmer avec l'Idée 4 qui semble dire que cela n'est pas si clair que cela.

PROPOSITION (n<=7). On a bien un arbre de profondeur au plus 1+2√2 pour n <= 7 points placés sur un cerle unité (source au centre).

PREUVE. Notons q_i le point diamétralement opposé au point p_i. Il existe toujours un point p_i tel que l'axe [p_i,q_i] coupe en deux l'ensemble des points. S'il y a des points confondus en p_i ou q_i, on peut répartir chaque demi-cercle de sorte qu'il y a au plus n/2 points de part et d'autres de p_i. En particulier, pour n <= 7, il y aura 3 points au plus dans chaque demi-cercle.

Pour voir qu'on peut couper en deux, on considère un axe [p_i,q_i] et on affecte les points à chaque demi-cercle: x(i) est le nombre de points en tournant à droite de p_i et y(i) le nombre de points affecté en tournant à droite de q_i. S'il y a des points multiples où diamétralement opposés, on les répartis au mieux en cassant la symétrie vers le demi-cercle à droite par exemple. S'il n'y a pas équilibre, on tourne l'axe à droite jusqu'à atteindre un nouveau point qui est soit p_{i+1}, soit q_j pour un certain j. À chaque mouvement, on peut s'arranger pour que x(i) évolue d'au plus d'une unité. D'autre part, après un demi-tour, disons un départ en p_i et une arrivée en q_i, x(i) et y(i) ont des valeur inversées. Il suit qu'à un moment donné |x(j)-y(j)| <= 1 pour un certain j. 

On choisit p_i qui coupe en deux le nombre n de points, donc avec 3 points au maximum dans chaque demi-cercle (en enlevant p_i). Dans chaque demi-cercle un arbre de hauteur deux suffit, donc de longueur au plus 2√2. Cela fait donc au plus 1+2√2.
QED.

Reste donc les cas avec n=8,9,10,11,12,13.

PROPOSITION (Cercle majorant). Soit P un ensemble de n points en position convexe et une source s (pas dans P) à distance au plus un de tous les points de P. Alors il existe un ensemble P' de n points placé sur le cercle de rayon unité et de centre s tel que OPT(P',s) >= OPT(P,s).

    C'est vrai pour L_2, mais peut-être généralisable pour d'autre métrique. C'est vrai pour L_1 et L_max car les cercles sont des carrés et les enveloppes convexes sont des rectangles. La transformation de P en P' est alors trivial. Notons que dans la définition d'un ensemble convexe, il faut que TOUS les plus courts chemins entre deux points soient dans le convexe.

PREUVE. Soit P = (p_0,...,p_{n-1}) les points en position convexe ordonnés selon l'envelopper convexe dans le sens direct. Sans perte de généralité, en décalant P et s d'un certain vecteur, on peut supposer que s est placée au centre d'un cercle de rayon unité contenant P. On va aussi supposer n >= 3, puisque pour 1,2 le résultat est trivial avec P'=P, OPT(P,s) = dist(s,{p_0}). Donc chaque point p_i a un successeur, p_{i+1} et prédécesseur p_{i-1}, les indices étant modulo n.

Pour chaque point p_i de P, on définit la demi-droite D_i^- (resp. D_i^+) issue de p_i obtenue en effectuant une rotation autour de p_i de 𝜋/4 (resp. -𝜋/4) du segment [p_i,p_{i-1}] (resp. [p_i,p_{i+1}]). Enfin, on pose cone(p_i) la région délimitée par les demi-droites D_i^- et D_i^+.

La propriété fondamentale des cones ainsi définis est:

        pour tout p_i, p_j de P, dist(cone(p_i),cone(p_j)) >= dist(p_i,p_j)

C'est liée à la convexité de P. Supposons j>i. La demi-droite D_i^+ est le 2e bord de cone(p_i) (2e par rapport au sens direct, D_i^- étant le 1er) alors que D_j^- est le 1er bord de cone(p_j). À cause de la convexité de P, les demi-droites D_i^+ et D_j^- forment un angle > 0 (il est égale à 0 si j=i+1). Si l'angle était < 0, les points entre p_i et p_j ne pourraient être en position convexe. Donc cone(p_i) et cone(p_j) ne peuvent intersecter la bande délimiter par ces deux demi-droites et le segment [p_i,p_j]. Leur distance est donc au moins dist(p_i,p_j), la longueur du segment [p_i,p_j].

On calcule P' de la manière suivante: pour chaque i, p'_i est n'importe quel point situer à l'intersection du cercle unité et centré sur s avec cone(p_i). Les points de P' sont donc sur le cercle unité centré sur s. Soit T' l'arbre solution optimal pour (P',s). On construit l'arbre T pour (P,s) à partir de T' en remplaçant chaque p'_i par p_i. Notons que p'_i est dans cone(p_i) et p'_j dans cone(p'_j). La profondeur de T est nécessairement <= profondeur de T car pour chaque arête du type p'_i-p'_j de T', dist(p_i,p_j) <= dist(p'_i,p'_j) à cause de la propriété sur les cones. Et donc l'arête p_i-p_j dans T est plus courte. Enfin, c'est aussi vrai pour la 1ère arête de l'arbre de T, soit s-p_i pour un certain i, car dist(s,p_i) <= 1 par hypothèse alors que dist(s,p'_i) = 1 par construction. C'est vrai pour toutes les arêtes de T.
QED

On va montrer un résultat intéressant, qui semble aussi connu, mais avec des preuves plus ou moins foireuses, comme souligné par https://les-mathematiques.net/vanilla/index.php?p=discussion/comment/1021853#Comment_1021853). Elle découle de la proposition du "cercle majorant".

PROPOSITION (Périmètre maximum d'un polygone non-régulier). Soit P un ensemble de points en position convexe dans le disque de rayon unité. Alors le périmètre de P est au plus 2|P|*sin(𝜋/|P|), ce qui est atteint pour un polygone régulier.

SKETCH DE PREUVE. La première partie consiste à pousser, depuis le centre du cercle contenant P, les points qui ne sont pas déjà sur le cercle et de dire que le périmètre a augmenté. Il faut utiliser la même transformation que dans la proposition du "cercle majorant" basé sur l'inéquation des cones dans le cas limité de p_i et p_{i+1}. Ensuite, on applique la proposition sur la somme des sinus, puisque le périmètre n'est qu'une somme de longeur de corde ou encore de sinus. C'est apparemment la deuxième étape qui n'est pas facile à démontrer de manière géométrique, c'est-à-dire sans le résultat analytique sur la somme des sinus, car la plupart des modifications géométriques, qui sont locales, ne permettre pas d'atteindre tous les polygones réguliers.
QED

Idée 4. Malheureusement, on ne peut pas extrapoler le résultat sur le périmètre pour la profondeur des arbres, mêmes s'ils sont planaires et monotones. La longueur des branches est effectivement des sommes de sinus <= 2𝜋, d'angles positifs s'ils sont monotones et de somme <= 2𝜋, mais on doit considérer le max de plusieurs branches, soient max{ sum_i sin(a_i), sum_j sin(b_j) }. Ces branches ont en général qu'un court préfixe et des branches ayant << n points. D'ailleurs, la proposition "un fils" affirme qu'un arbre optimal est de hauteur ceil(log_2(n)) [C'est faux]. Aucune raison que le max soit atteint pour a_i = b_j = 2𝜋/n. Ceci dit, pour un arbre solution binaire complet (sauf la racine), donc avec n = 2^k points (en comptant la source), on peut peut-être dire quelque chose ... En tout cas, il est faux de dire que pour n = 2^k points, l'arbre optimal est nécessairement binaire complet (cf. contre-exemple ci-dessus).

Idée 5. On pourrait modifier les arbres pour les rendre moins profonds, quitte à supposer n plus grand, disons n >= 8. Dans le ]A,B] non vide, au lieu d'informer p_{k+1}, on pourrait viser un p_j (j>k) proche du "milieu" de ]A,B] et profiter de la longueur du dernier arc ]p_j,B] pour informer en parallèle l'arc ]A,p_j[. L'arbre est encore planaire mais plus monotone, mais cela pourrait potentiellement permettre d'augmenter encore un peu l'angle t0 pour que les branches p_{i+1} - p_k, p_{i+1} - p_{k+1} - A et p_{i+1} - p_{k+1} - B soient a peu près de même longueur. Notons que pour que l'arbre solution puisse faire un branchement (binaire) il faut atteindre un robot. Viser un point sans robot est possible, mais il ne permet pas de faire de branchement.

Idée 6. Cela serait cool de montrer que pour n points sur un cercle, le pire des cas est le cas uniforme. Ce n'est pas si clair que cela, bien que les expériences montrent que cela semble vrai. La difficulté est qu'on ne peut pas se baser sur une preuve qui transformerait les points initial P en points uniforme P' tout en augmentant les distances et en gardant l'ordre cyclique des points (donc avec la transformation inverse on en déduit un arbre optimal plus court pour P que pour P'). Le problème vient que pour certains points en position convexe il peut y avoir des croisements. Ceci dit on avait le problème avant, et la preuve marche pourtant ...

Idée 7. Cela serait cool de montrer que le pire cas est pour n>=4 pair. L'idée est que lorsque n est pair, on ne peut pas avoir un arbre binaire complet. On peut donc enlever un point et se ramener au cas impair ... [À VOIR].

CONJECTURE (Parité). Pour tout entier k>=1, alpha(2k-1) <= alpha(2k), où alpha(n) est la profondeur optimale de tout ensemble de n points du plan (euclidien), hors la source, à distance <= 1 de la source.

On aimerait bien aussi démontrer que pour k>=2, alpha(2k+2) < alpha(2k) et alpha(2k+3) < alpha(2k+1), mais rien n'est moins sûr à la vue des anomalies de T_segment(n) (pour n points uniformes sur un cercle) vers n = 60,...,66.

La conjecture semble vérifiée pour les points uniformément distribués sur un cercle. Cela ne prouve rien bien sûr. On remarque que lorsque n=2k est pair, il y a forcément un sommet de l'arbre qui est de degré deux. Idée pour prouver cette conjecture. On choisit un ensemble P de n=2k-1 points en positions critiques, c'est-à-dire dont l'arbre optimal T est de profondeur alpha(n). On pose r = alpha(n)-alpha(n+1) - eps avec eps>0 assez petit. On suppose (par l'absurde) que r>0. On choisit d'ajouter un point p' à cet l'ensemble, passant à n+1 points. On calcule alors l'arbre T' optimal pour ces n+1 points. On supprime p' de T' qu'on tranforme en un arbre T'' pour l'ensemble initial P avec depth(T'') <= depth(T')+r.

C'est une contradiction car d'une part: alpha(n) <= depth(T''), et d'autre part depth(T')+r = alpha(n+1) + alpha(n)-alpha(n+1)-eps = alpha(n) - eps < alpha(n), soit alpha(n) < alpha(n). Il faut donc deux étapes: (1) ajouter p'; (2) transformer T' en T'' en supprimant p'. On peut placer p' à distance r d'un des points de P, typiquement sur une arête u-v de l'arbre optimal T. Si dans T', deg(p') = 1, alors on peut supprimer p' qui est une feuille, et T'' est un arbre de profondeur <= depth(T). Si deg(p') = 2, alors on peut ajouter une arête entre ses deux voisins pour obtenir un arbre T'' de profondeur <= depth(T). Reste donc le cas où deg(p')=3. Si deux des trois voisins de p' dans T' sont u et v, alors on peut contracter l'arête p'-u ou p'-v de longueur r et obtenir un arbre T'' pour P de profondeur <= depth(T')+r. Le cas problématique est donc lorque p' possède 3 voisins dans T' sans contenir u et v. Bien sûr cela reste vrai pour tout placement de p' autour de u (et à distance <= 1 de s). 

Le problème est qu'on ne peut pas dire grand chose sur les arêtes qui doivent ou ne doivent pas être dans un arbre optimal, même (et surtout) lorsque les points sont sur un cercle de rayon 1. On aimerait bien pouvoir dire par exemple que u-v sont dans l'arbre optimal (où qu'il existe un arbre optimal qui contient u-v) si u et v sont les plus proches. Mais c'est faux. Les deux points les plus proches ne sont pas (leur arête) forcément l'arbre optimal, même s'ils sont alignés: 1--s----2-----3. L'arbre optimal est s->2->{1,3} de profondeur 10. L'arbre contenant s->1->{2,3} serait de profondeur 11. On peut aussi imaginer trois points 1,2,3 qui forment un Y depuis s (avec la fourche en 1) mais avec 2 et 3 très très proches. On peut même imaginer que 1,2,3 soient sur un cercle de rayon 1 de centre s. Autre exemple: il est possible que les deux points les plus éloignés (leur arête) soient dans l'arbre optimal: A---s---B où A et B sont une superposition de n/2 points.
"""
