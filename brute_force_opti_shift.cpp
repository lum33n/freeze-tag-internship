#include<stdlib.h>
#include<iostream>
#include<unordered_map>
#include<cmath>

using namespace std;

double max(double a, double b){
    if (b <= a) return a;
    return b;
}

class opt_set{
public:
    unsigned long long i;
    int n;
    int beg;
    bool b;
    opt_set(int n){
        this->n = n;
        beg = 0;
        i = 0;
        b = 0;
    }
    void set(int k){
        i |= (1llu << k);
    }
    void reset(int k){
        i &= ~(1llu << k);
    }
    void rotate(int k){
        i = (i >> k) | ((i << (n - k)) & ((1llu << n) - 1llu));
    }
    int log(){
        return __builtin_ctzll(i);
    }
    int count(){
        return __builtin_popcountll(i);
    }
    int operator[](int k){
        return (i >> k)%2;
    }
    bool operator==(const opt_set& el) const {
        return ((i >> beg) | ((i << (n - beg)) & ((1llu << n) - 1llu))) == ((el.i >> el.beg) | ((el.i << (el.n - el.beg)) & ((1llu << el.n) - 1llu))) and (el.b == b);
    }
    unsigned long long hash(){
        return i + beg * 930472387 + b * 34029489980;
    }
};

struct hashopt_set{
    unsigned long long operator()(const opt_set& el) const {
        return ((el.i >> el.beg) | ((el.i << (el.n - el.beg)) & ((1llu << el.n) - 1llu))) + (unsigned long long)el.b * 3042973024;
    }
};

double pre_bruteforce_opt(int n, opt_set& ens, unordered_map<opt_set, double, hashopt_set>& cache, double* chord_cache){
    int count = ens.count();
    if (count == 1){
        return 0;
    }
    if (count == 2){
        int temp;
        ens.reset(ens.beg);
        temp = ens.log();
        ens.set(ens.beg);
        return chord_cache[abs(ens.beg - temp)];
    }

    if (const auto& it = cache.find(ens); it != cache.end()){
        return it->second;
    }

    double res = INFINITY;
    double v;

    if (!ens.b){
        opt_set X(n);
        X.b = true;
        X.i = ens.i;
        X.reset(ens.beg);
        for (int beg2 = 0; beg2 < n; beg2++){
            if (X[beg2]){
                X.beg = beg2;
                v = pre_bruteforce_opt(n, X, cache, chord_cache);
                v += chord_cache[abs(ens.beg - beg2)];
                if (v < res) res = v;
            }
        }
        cache[ens] = res;
        return res;
    }
    //now we have 2 robots

    double res1, res2;
    int c;
    opt_set X(n);
    opt_set Xb(n);
    X.b = false;
    Xb.b = false;
    X.beg = ens.beg;
    Xb.beg = ens.beg;

    for (int i = 0; i < (1llu << (count-1)); i++){

        c = 0;
        X.i = 0;
        Xb.i = 0;
        for (int j = 0; j < n; j++){
            if (ens[j] == 1 and ens.beg != j){
                if ((i >> c)%2 == 1){
                    X.set(j);
                } else {
                    Xb.set(j);
                }
                c++;
            }
        }

        X.set(ens.beg);
        Xb.set(ens.beg);

        res1 = pre_bruteforce_opt(n, X, cache, chord_cache);
        if (res1 >= res) continue;
        res2 = pre_bruteforce_opt(n, Xb, cache, chord_cache);
        if (max(res1, res2) < res) res = max(res1, res2);
    }
    cache[ens] = res;
    return res;
}

double bruteforce_opt(int n){
    unordered_map<opt_set, double, hashopt_set> cache;
    double* chord_cache = (double*)malloc(sizeof(double)*n);
    for (int i = 0; i < n; i++){
        chord_cache[i] = 2*sin(M_PI*i/n);
    }
    opt_set ens(n);
    ens.i = (1llu << n) - 1;
    ens.beg = 0;
    ens.b = true; //two robots
    double res = pre_bruteforce_opt(n, ens, cache, chord_cache);
    free(chord_cache);
    return res+1;
}

int main(){
    cout << bruteforce_opt(17) << endl;
    return 0;
}
