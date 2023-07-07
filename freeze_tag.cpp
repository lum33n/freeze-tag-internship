#include<iostream>
#include<stdlib.h>
#include<math.h>
#include<vector>
#include<random>
#include<time.h>
#include<algorithm>
#include<unordered_map>
#include<map>
#include<set>

class opt_set{
public:
    unsigned long long n;
    int beg;
    int count;
    opt_set(){
        n = 0;
        beg = 0;
        count = 0;
    }
    unsigned long long mask(int k){
        return 1llu << k;
    }
    void set(int k){ //only used on false values
        n ^= mask(k);
        count++;
    }
    void reset(int k){ //only used on true values
        n ^= mask(k);
        count--;
    }
    void clear(){
        n = 0;
        count = 0;
        beg = 0;
    }
    int log(){ // there must be an element
        return __builtin_ctzll(n);
    }
    int operator[](int k){
        return (n >> k)%2;
    }
    void rotate(int k, int nbits){
        // n =  0b a b c d e
        // k = 2
        // nbits = 5
        // -> 0b d e a b c = ((0b a b c d e) >> 2) | (((0b a b c d e) & ((1 << 2) - 1)) << (5 - 2))
        n = (n >> k) | ((n & ((1llu << k) - 1)) << (nbits - k));
        //for (int i = 0; i < k; i++){
        //    if (n%2 == 1){
        //        n >>= 1;
        //        n += mask(nbits-1);
        //    } else {
        //        n >>= 1;
        //    }
        //}
    }
    void shift(int nbits) {
        rotate(beg, nbits);
    }
    bool operator==(const opt_set& a) const {
        return a.n == n and a.beg == beg;
    }
    bool operator<(const opt_set& other) const {
        return (n < other.n) or (n == other.n and beg < other.beg);
    }
    long long has(){
        return n + beg*30943298479;
    }
};

void sort(float* tab, int n){
    float t;
    for (int i = 0; i < n-1; i++){
        for (int j = 0; j < n-i-1; j++){
            if (tab[j] > tab[j+1]){
                t = tab[j];
                tab[j] = tab[j+1];
                tab[j+1] = t;
            }
        }
    }
}

template <typename t>
void shuffle(std::vector<t>& vec, int a, int b){
    for (int i = a; i < b; i++){
        int c = rand()%(b-a) + a;
        t k = vec[i];
        vec[i] = vec[c];
        vec[c] = k;
    }
}

class point{
public:
    float x;
    float y;
    point(){

    }
    point(float x, float y){
        this->x = x;
        this->y = y;
    }
    void add(point& v){
        this->x += v.x;
        this->y += v.y;
    }
    void sub(point& v){
        this->x -= v.x;
        this->y -= v.y;
    }
    void show(){
        std::cout << "(" << x << ", " << y << ")";
    }
};

class graph{
public:
    std::vector<point> node_list;

    graph(){
        node_list = {};
    }
    void generate_conv(int n){
        node_list = {};
        std::vector<float> X;
        X.resize(n);
        std::vector<float> Y;
        Y.resize(n);
        for (int i = 0; i < n; i++){
            X[i] = ((float)rand())/RAND_MAX;
            Y[i] = ((float)rand())/RAND_MAX;
        }
        std::sort(X.begin(), X.end());
        std::sort(Y.begin(), Y.end());
        shuffle(X, 1, X.size()-1);
        shuffle(Y, 1, Y.size()-1);
        std::vector<float> vectorx = {};
        std::vector<float> vectory = {};
        std::vector<float> firstchainx = {};
        std::vector<float> firstchainy = {};
        std::vector<float> secondchainx = {};
        std::vector<float> secondchainy = {};
        for (int i = 1; i < n/2; i++){
            firstchainx.push_back(X[i]);
            firstchainy.push_back(Y[i]);
        }
        for (int i = n/2; i < n-1; i++){
            secondchainx.push_back(X[i]);
            secondchainy.push_back(Y[i]);
        }
        vectorx.push_back(firstchainx[0] - X[0]);
        vectorx.push_back(X[0] - secondchainx[0]);
        vectorx.push_back(X[n-1] - firstchainx[firstchainx.size() - 1]);
        vectorx.push_back(secondchainx[secondchainx.size() - 1] - X[n-1]);
        for (int p = 1; p < firstchainx.size(); p++){
			vectorx.push_back((firstchainx[p] - firstchainx[p-1]));
        }
		for (int p = 1; p < secondchainx.size(); p++){
			vectorx.push_back((secondchainx[p-1] - secondchainx[p]));
        }
        vectory.push_back(firstchainy[0] - X[0]);
        vectory.push_back(X[0] - secondchainy[0]);
        vectory.push_back(X[n-1] - firstchainy[firstchainy.size() - 1]);
        vectory.push_back(secondchainy[secondchainy.size() - 1] - X[n-1]);
        for (int p = 1; p < firstchainy.size(); p++){
			vectory.push_back((firstchainy[p] - firstchainy[p-1]));
        }
		for (int p = 1; p < secondchainy.size(); p++){
			vectory.push_back((secondchainy[p-1] - secondchainy[p]));
        }
        shuffle(vectory, 0, vectory.size());
        std::vector<point> vectors = {};
        for (int i = 0; i < vectory.size(); i++){
            vectors.push_back(point(vectorx[i], vectory[i]));
        }
        std::sort(vectors.begin(), vectors.end(), [](point& x, point& y){
            return (std::atan2(x.x, x.y) <= std::atan2(y.x, y.y));
        });
        point beg(0, 0);
        for (int i = 0; i < n;i++){
            beg.add(vectors[i]);
            node_list.push_back(beg);
        }
    }
    void generate_angle(std::vector<float>& anglelist){
        node_list.resize(anglelist.size());
        for (int i = 0; i < anglelist.size(); i++){
            node_list[i].x = std::cos(anglelist[i]);
            node_list[i].y = std::sin(anglelist[i]);
        }
    }
    void generate_unif(int n){
        std::vector<float> al = {};
        al.resize(n);
        for (int i = 0; i < n; i++){
            al[i] = ((float)((float)i * (float)2*(float)M_PI)) / n;
        }
        generate_angle(al);
    }
    void show(){
        std::cout << "----------------------------" << std::endl;
        std::cout << "[" << std::endl;
        for (int i = 0; i < node_list.size(); i++){
            node_list[i].show();
            std::cout << "," << std::endl;
        }
        std::cout << "]" << std::endl;
        std::cout << "----------------------------" << std::endl;
    }
};

float distance(point& a, point& b){
    float delta_x = b.x - a.x;
    float delta_y = b.y - a.y;
    return std::sqrt(delta_x*delta_x + delta_y*delta_y);
}

struct sethasher{
    std::size_t operator()(const std::pair<std::set<int>, int>& el) const {
        long long res = 0;
        for (int const i: el.first){
            int x = i;

            x ^= x >> 17;
            x *= 830770091;   // 0xed5ad4bb
            x ^= x >> 11;
            x *= -1404298415; // 0xac4c1b51
            x ^= x >> 15;
            x *= 830770091;   // 0x31848bab
            x ^= x >> 14;

            res += x;
        }

        res += el.second * 30293592759;
        return res;
    }
};

float max(float a, float b){
    if (a >= b) return a;
    return b;
}

float precalc_monalgo(graph& G, std::set<int>& ens, int beg, std::unordered_map<std::pair<std::set<int>, int>, float, sethasher>& cache){
    if (ens.size() <= 1){
        return 0;
    }
    if (ens.size() == 2){
        int temp[2];
        int i = 0;
        for (int const a: ens){
            temp[i] = a;
            i++;
        }
        return distance(G.node_list[temp[0]], G.node_list[temp[1]]);
    }
    auto cacheind = std::make_pair(ens, beg);
    if (cache.find(cacheind) != cache.end()){
        return cache[cacheind];
    }
    float res = INFINITY;
    std::set<int> X = {};
    std::set<int> Xb = {};
    int a = 0;
    int b = 1;
    int i;
    while (a < ens.size()-1){
        i = 0;
        X.clear();
        Xb.clear();

        for (const int el: ens){
            if (el != beg){
                if (a <= i and i < b){
                    X.insert(el);
                } else {
                    Xb.insert(el);
                }
            }
            i++;
        }
        b++;
        if (b >= ens.size()){
            a++;
            b = a+1;
        }

        float res1 = INFINITY;
        float res2 = INFINITY;
        float v1;
        float v2;
        //now we have X and Xb ready
        for (const int beg2: X){
            v1 = precalc_monalgo(G, X, beg2, cache);
            v1 += distance(G.node_list[beg], G.node_list[beg2]);
            if (v1 < res1){
                res1 = v1;
            }
        }
        for (const int beg2: Xb){
            v2 =precalc_monalgo(G, Xb, beg2, cache);
            v2 += distance(G.node_list[beg], G.node_list[beg2]);
            if (v2 < res2){
                res2 = v2;
            }
        }
        if (max(res1, res2) < res){
            res = max(res1, res2);
        }
    }
    cache.emplace(cacheind, res);
    return res;
}

float monalgo(graph& G){
    int n = G.node_list.size();
    std::unordered_map<std::pair<std::set<int>,int>, float, sethasher> cache;
    std::set<int> ens = {};
    for (int i = 0; i < n; i++){
        ens.insert(i);
    }
    float res = INFINITY;
    float v;
    for (int i = 0; i < n; i++){
        v = precalc_monalgo(G, ens, i, cache);
        if (v < res){
            res = v;
        }
    }
    return res+1;
}

float precalc_bruteforce(graph& G, std::set<int>& ens, int beg, std::unordered_map<std::pair<std::set<int>, int>, float, sethasher>& cache){
    if (ens.size() <= 1){
        return 0;
    }
    if (ens.size() == 2){
        int temp[2];
        int i = 0;
        for (int const a: ens){
            temp[i] = a;
            i++;
        }
        return distance(G.node_list[temp[0]], G.node_list[temp[1]]);
    }
    auto cacheind = std::make_pair(ens, beg);
    if (cache.find(cacheind) != cache.end()){
        return cache[cacheind];
    }
    float res = INFINITY;
    std::set<int> X = {};
    std::set<int> Xb = {};
    int a = 0;
    int i;
    while (a < (1 << ens.size())){
        i = 0;
        X.clear();
        Xb.clear();

        for (const int el: ens){
            if (el != beg){
                if ((a >> i)%2 == 1){
                    X.insert(el);
                } else {
                    Xb.insert(el);
                }
            }
            i++;
        }
        a++;

        float res1 = INFINITY;
        float res2 = INFINITY;
        float v1;
        float v2;
        //now we have X and Xb ready
        for (const int beg2: X){
            v1 = precalc_bruteforce(G, X, beg2, cache);
            v1 += distance(G.node_list[beg], G.node_list[beg2]);
            if (v1 < res1){
                res1 = v1;
            }
        }
        for (const int beg2: Xb){
            v2 =precalc_bruteforce(G, Xb, beg2, cache);
            v2 += distance(G.node_list[beg], G.node_list[beg2]);
            if (v2 < res2){
                res2 = v2;
            }
        }
        if (max(res1, res2) < res){
            res = max(res1, res2);
        }
    }
    cache.emplace(cacheind, res);
    return res;
}

float bruteforce(graph& G){
    int n = G.node_list.size();
    std::unordered_map<std::pair<std::set<int>,int>, float, sethasher> cache;
    std::set<int> ens = {};
    for (int i = 0; i < n; i++){
        ens.insert(i);
    }
    float res = INFINITY;
    float v;
    for (int i = 0; i < n; i++){
        v = precalc_bruteforce(G, ens, i, cache);
        if (v < res){
            res = v;
        }
    }
    return res+1;
}

void compare_monalgo_bruteforce(int n){
    auto c = clock();
    graph G;
    auto init = clock();
    G.generate_conv(n);
    //std::cout << "  gen done for n = " << G.node_list.size() << std::endl;
    float v1 = bruteforce(G);
    //std::cout << "  brute force done" << std::endl;
    float v2 = monalgo(G);
    //std::cout << "  monalgo done" << std::endl;
    if (v1 < v2){
        std::cout << "-----------------------------------" << std::endl;
        std::cout << "alerte rouge, contre exemple trouve" << std::endl;
        std::cout << "brute_force : " << v1 << "   monalgo : " << v2 << std::endl;
        G.show();
        std::cout << "-----------------------------------" << std::endl;
    }
    std::cout << "one comparison done in " << ((double)clock() - (double)c)/CLOCKS_PER_SEC << " s" << std::endl;
}

double chord(int n, int k){
    //on a circle of n points, the length of a chord to get k points further
    return abs(2*std::sin(k*M_PI/n));
}

double segment_cm_pre(int n, int k, double* cache){
    if (k == 0) return 0;
    if (k == 1) return chord(n, 1);
    if (cache[k] != -1.) return cache[k];
    double m = INFINITY;
    double m1;
    double m2;
    for (int t = 2; t <= k; t++){
        m1 = segment_cm_pre(n, t-2, cache);
        m2 = segment_cm_pre(n, k - t, cache);
        m1 = max(m1 + chord(n, 1), m2 + chord(n, t));
        if (m1 < m) m = m1;
    }
    cache[k] = m;
    return m;
}

double segment_cm(int n){
    double* cache = (double*)malloc(sizeof(double)*(n+1));
    for (int i = 0; i < n+1; i++){
        cache[i] = -1.;
    }
    double res = segment_cm_pre(n, std::ceil((n-3)/2), cache);
    free(cache);
    return res+1 + chord(n, 1);
}

double segment_c_pre(int n, int k1, int k2, double* cache){
    if (cache[k1*(n+1) + k2] != -1.) return cache[k1*(n+1) + k2];
    if (k1 == 0 and k2 == 0){
        cache[0] = 0;
        return 0;
    }
    double m = INFINITY;
    if (k1 > 0 and k2 > 0){
        double x1min = INFINITY;
        double x2min = INFINITY;
        for (int t = 1; t < k1+1; t++){
            double x = segment_c_pre(n, k1-t, t-1, cache); x += chord(n, t);
            if (x < x1min) x1min = x;
        }
        for (int t = 1; t < k2+1; t++){
            double x = segment_c_pre(n, t-1, k2-t, cache); x += chord(n, t);
            if (x < x2min) x2min = x;
        }
        cache[k1*(n+1) + k2] = max(x1min, x2min);
        return cache[k1*(n+1) + k2];
    }else{
        int s = 1;
        if (k1 > 0){
            auto temp = k1;
            k1 = k2;
            k2 = temp;
            s = -1;
        }

        if (k2 == 1){
            cache[k1*(n+1) + k2] = chord(n, 1);
            cache[k2 * (n+1) + k1] = chord(n, 1);
            return cache[k1*(n+1) + k2];
        }

        for (int t1 = 1; t1 < k2; t1++){
            double c1 = chord(n, t1);
            for (int t2 = t1+1; t2 < k2+1; t2++){
                double c2 = chord(n, t2);
                int k = t2 - t1 -1;
                for (int i = 0; i < k+1; i++){
                    double x1; if (s > 0) x1 = segment_c_pre(n, t1 -1, i, cache); if (s < 0) x1 = segment_c_pre(n, i, t1-1, cache);
                    double x2; if (s > 0) x2 = segment_c_pre(n, k-i, k2-t2, cache); if (s < 0) x2 = segment_c_pre(n, k2-t2, k-i, cache);
                    x1 = max(x1 + c1, x2 + c2);
                    if (x1 < m) m = x1;
                }
            }
        }
        cache[k1*(n+1) + k2] = m;
        cache[k2 * (n+1) + k1] = m;
        return m;

    }
}

double segment_c(int n){
    double* cache = (double*)malloc(sizeof(double)*(n+1)*(n+1));
    for (int i = 0; i < (n+1)*(n+1); i++){
        cache[i] = -1.;
    }
    int k = std::ceil((n-1)/2);
    double res = segment_c_pre(n, k, n-1-k, cache);
    free(cache);
    return res+1;
}

bool next_rec_set(int sum_value, int* rec_set, int k){
    if (k <= 1) return false;
    if (next_rec_set(sum_value - rec_set[0], rec_set+1, k-1)){
        return true;
    } else {
        if (rec_set[0] == 0) return false;
        rec_set[0]--;
        rec_set[k-1] = 0;
        rec_set[1] = sum_value - rec_set[0];
        return true;
    }
}

//warning, this does not work beware
template<int k>
double multi_segment_c_pre(int n, int ki[k], double* cache){
    //cache of length (n+1)**k
    int cacheind = 0;
    int mul = 1;
    for (int i = 0; i < k; i++){
        cacheind += mul*ki[i];
        mul *= (n+1);
    }
    if (cache[cacheind] != -1.) return cache[cacheind];
    double mi[k];
    int rec_set[k];
    for (int i = 0; i < k; i++){
        if (ki[i] <= 1){
            mi[i] = 0;
            continue;
        }
        mi[i] = INFINITY;
        double current_max;
        for (int j = 0; j < k; j++){
            rec_set[j] = 0;
        }
        rec_set[0] = ki[i]-1;
        current_max = multi_segment_c_pre<k>(n, rec_set, cache);
        if (current_max < mi[i]) mi[i] = current_max;
        while (next_rec_set(ki[i], rec_set, k)){
            //rec_set contains every possible ways of having sum rec_set = k[i]
            current_max = multi_segment_c_pre<k>(n, rec_set, cache);
            if (current_max < mi[i]) mi[i] = current_max;
        }
    }
    double res = INFINITY;
    double timeR1, timeR2;
    //now we need to find the best possible way to combine the m[i] into two subgroups for two robots to work on their stuff.
    for (int i = 0; i < 1 << k; i++){
        timeR1 = 0;
        timeR2 = 0;
        int last_robot_pos1 = 0;
        int last_robot_pos2 = 0;
        int sum = 1;
        double chord_distance1 = 0;
        double chord_distance2 = 0;
        for (int j = 0; j < k; j++){
            if (ki[j] == 0){
                continue;
            }
            if ((i/(1 << j))%2 == 1){
                chord_distance1 += chord(n, sum - last_robot_pos1);
                timeR1 += mi[j] + chord_distance1;
                last_robot_pos1 = sum;
            } else {
                chord_distance2 += chord(n, sum - last_robot_pos2);
                timeR2 += mi[j] + chord_distance2;
                last_robot_pos2 = sum;
            }
            sum += ki[j];
        }
        std::cout << timeR1 << " " << timeR2 << std::endl;
        if (max(timeR1, timeR2) < res){
            res = max(timeR1, timeR2);
        }
    }
    cache[cacheind] = res;
    return res;
}

template<int k>
double multi_segment_c(int n){
    int sum = 1;
    for (int i = 0; i < k; i++){
        sum *= n+1;
    }
    double* cache = (double*)malloc(sizeof(double)*sum);
    for (int i = 0; i < sum; i++){
        cache[i] = -1.;
    }
    double res = INFINITY;
    int ki[k];
    for (int i = 0; i < k; i++) ki[i] = 0;
    ki[0] = n-1;
    double mi = INFINITY;
    while (next_rec_set(n, ki, k)){
        double temp = multi_segment_c_pre<k>(n, ki, cache);
        if (temp < mi) mi = temp;
    }
    free(cache);
    return mi + 1;
}

class opt_set2{
public:
    unsigned long long n;
    int beg;
    opt_set2(){
        n = 0;
        beg = 0;
    }
    void clear(){
        n = 0;
        beg = 0;
    }
    void rotate(int k, int nbits){
        n = ((n >> k) | ((n << (nbits - k)) & ((1llu << nbits) -1llu) ));
    }
    void shift(int nbits){
        rotate(beg, nbits);
    }
    int log(){
        return __builtin_ctzll(n);
    }
    int count(){
        return __builtin_popcountll(n);
    }
    bool operator==(const opt_set2& other) const{
        return other.n == n and other.beg == beg;
    }
    int operator[](const int k) const {
        return (n >> k)%2;
    }
    void set(int k){
        n |= (1llu << k);
    }
    void reset(int k){
        n &= ~(1llu << k);
    }
};

struct sethasher3{
    std::size_t operator()(const opt_set2& el) const {
        return el.n + el.beg * 3902830948;
    }
};

struct sethasher2{
    std::size_t operator()(const opt_set& el) const {
        return el.n + el.beg * 3902830948;
    }
};

template<int k>
double pre_multi_segment_opt(int n, opt_set2& ens, std::unordered_map<opt_set2, double, sethasher3>& cache, double* chord_cache){
    ens.shift(n);

    if (const auto & it = cache.find(ens); it != cache.end()){
        return it->second;
    }
    int count = ens.count();
    if (count <= 1){
        cache[ens] = 0;
        return 0;
    }
    if (count == 2){
        int temp;
        ens.reset(0);
        temp = ens.log();
        ens.set(0);
        double res = chord_cache[temp];
        cache[ens] = res;
        return res;
    }
    double res = INFINITY;
    double res1, res2;
    double v;
    int count1, count2;
    int sum;
    int beg2, temp;
    opt_set2 X;
    opt_set2 Xb;

    int ki[2*k];
    for (int i = 0; i < k; i++){
        ki[2*i] = 0;
        ki[2*i + 1] = 0;
    }
    ki[0] = n;
    while (next_rec_set(n, ki, 2*k)){
        X.clear();
        Xb.clear();
        sum = 0;
        for (int i = 0; i < k; i++){
            X.n |= ens.n & ((1llu<<(ki[2*i] + sum)) - ( 1llu << sum));
            sum += ki[2*i];
            Xb.n |= ens.n & (( 1llu << (ki[2*i + 1] + sum)) - (1llu << sum));
            sum += ki[2*i+1];
        }
        X.n &= ~1llu;
        Xb.n &= ~1llu;

        if (X.n == 0 or Xb.n == 0) {continue;};

        res1 = INFINITY;
        res2 = INFINITY;

        temp = X.log();
        beg2 = temp;
        if (X.count() == 1){
            v = chord_cache[beg2];
            if (v < res1) res1 = v;
        } else {
        for (; beg2 < n; beg2++){
            if (X[beg2]){
            X.beg = beg2;
            v = pre_multi_segment_opt<k>(n, X, cache, chord_cache);
            v += chord_cache[beg2];
            if (v < res1) res1 = v;
            }
        }
        }
        beg2 = Xb.log();
        if (Xb.count() == 1){
            v = chord_cache[beg2];
            if (v < res2) res2 = v;
        } else {
        for (; beg2 < n; beg2++){
            if (Xb[beg2]){
                Xb.beg = beg2;
                v = pre_multi_segment_opt<k>(n, Xb, cache, chord_cache);
                v += chord_cache[beg2];
                if (v < res2) res2 = v;
            }
        }
        }

        if (max(res1, res2) < res) res = max(res1, res2);
    }
    cache[ens] = res;
    return res;
}

template<int k>
double multi_segment_opt_c(int n){
    std::unordered_map<opt_set2, double, sethasher3> cache;
    opt_set2 ens;
    double* chord_cache = new double[n];
    for (int i =0; i < n; i++){
        chord_cache[i] = chord(n, i);
    }
    ens.n = (1llu<<(n)) - 1;
    ens.beg = 0;
    double v = 1+pre_multi_segment_opt<k>(n, ens, cache, chord_cache);
    delete[] chord_cache;
    return v;
}

long long rip = 0;

double opt_precalc_monalgo(int n, opt_set& ens, std::unordered_map<opt_set, double, sethasher2>& cache, double* chord_cache){
    rip++;

    if (const auto & it = cache.find(ens); it != cache.end()){
        return it->second;
    }
    if (ens.count <= 1){
        cache[ens] = 0;
        return 0;
    }
    if (ens.count == 2){
        int temp;
        ens.reset(0);
        temp = ens.log();
        ens.set(0);
        double res = chord_cache[temp];
        cache[ens] = res;
        return res;
    }
    double res = INFINITY;
    opt_set X;
    opt_set Xb;
    int a = 1;
    int b = 2;
    int i;
    while (a < n - 1){
        i = 0;

        X.n = ens.n & ((1 << b) - (1 << a));
        X.count = __builtin_popcountll(X.n);
        X.beg = 0;
        Xb.n = (ens.n ^ X.n) - 1;
        Xb.count = ens.count - X.count - 1;
        Xb.beg = 0;

        b++;
        if (b > n){
            a++;
            b = a+1;
        }

        if (!X.n || !Xb.n) continue;

        double res1 = INFINITY;
        double res2 = INFINITY;
        double v1;
        double v2;

        //now we have X and Xb ready
        if (X.count == 1) {
            int beg2 = X.log();
            v1 = chord_cache[beg2];
            if (v1 < res1){
                res1 = v1;
            }
        } else {
            int beg2 = X.log();
            X.rotate(beg2, n);
            while (beg2 < n){
                v1 = opt_precalc_monalgo(n, X, cache, chord_cache);
                v1 += chord_cache[beg2];
                if (v1 < res1){
                    res1 = v1;
                }
                X.reset(0);
                int i = X.log();
                X.set(0);
                beg2 += i;
                X.rotate(i, n);
            }
        }
        if (Xb.count == 1) {
            int beg2 = Xb.log();
            v1 = chord_cache[beg2];
            if (v1 < res1){
                res1 = v1;
            }
        } else {
            int beg2 = Xb.log();
            Xb.rotate(beg2, n);
            while (beg2 < n){
                v2 = opt_precalc_monalgo(n, Xb, cache, chord_cache);
                v2 += chord_cache[beg2];
                if (v2 < res2){
                    res2 = v2;
                }
                Xb.reset(0);
                int i = Xb.log();
                Xb.set(0);
                beg2 += i;
                Xb.rotate(i, n);
            }
        }
        if (max(res1, res2) < res){
            res = max(res1, res2);
        }
    }
    cache.emplace(ens, res);
    return res;
}

double opt_monalgo(int n){//warning, never exceed n = 63
    std::unordered_map<opt_set, double, sethasher2> cache;
    opt_set ens;
    double* chord_cache = new double[n];
    for (int i =0; i < n; i++){
        chord_cache[i] = chord(n, i);
    }
    ens.n = (1llu<<(n)) - 1;
    ens.count = n;
    ens.beg = 0;
    double v = 1+opt_precalc_monalgo(n, ens, cache, chord_cache);
    delete[] chord_cache;
    return v;
}

int main(){
    std::srand((unsigned)time(NULL));

    std::cout << multi_segment_opt_c<3>(10) << std::endl;

    //std::cout<< opt_monalgo(22) << std::endl;
    //std::cout << rip << std::endl;

    // std::cout << "t for time based of n for number based s to test the time taken by monalgo" << std::endl;
    // char answ2;
    // std::cin >> answ2;
    // if (answ2 == 'n'){
    //     while (true){
    //         std::cout << "0 to exit and an int to continue" << std::endl;
    //         int answ;
    //         std::cin >> answ;
    //         if (answ == 0){
    //             std::cout << "exiting" << std::endl;
    //             break;
    //         }
    //         for (int i = 0; i < answ; i++){
    //             compare_monalgo_bruteforce(11);
    //         }
    //     }
    // } else if (answ2 == 't'){
    //     while (true){
    //         std::cout << "0 to exit and an int to continue" << std::endl;
    //         int answ;
    //         std::cin >> answ;
    //         if (answ == 0){
    //             std::cout << "exiting" << std::endl;
    //             break;
    //         }
    //         auto c = clock();
    //         while ((clock() - c) < CLOCKS_PER_SEC*answ){
    //             compare_monalgo_bruteforce(6);
    //         }
    //     }
    // } else if (answ2 == 's'){
    //     std::cout << "enter n" << std::endl;
    //     int answ;
    //     std::cin >> answ;
    //     graph G;
    //     G.generate_conv(answ);
    //     auto c = clock();
    //     float v = monalgo(G);
    //     std::cout << "value " << v << " in " << ((double)clock() - (double)c)/CLOCKS_PER_SEC << " s" << std::endl;
    // }
    return 0;
}
