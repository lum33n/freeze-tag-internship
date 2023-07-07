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
    bool b;
    opt_set(int n){
        this->n = n;
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
        return i == el.i and (el.b == b);
    }
    unsigned long long hash(){
        return i + b * 34029489980;
    }
};

struct hashopt_set{
    unsigned long long operator()(const opt_set& el) const {
        return el.i + (unsigned long long)el.b * 3042973024;
    }
};

double pre_bruteforce_opt(int n, opt_set& ens, unordered_map<opt_set, double, hashopt_set>& cache, double* chord_cache){
    int count = ens.count();
    if (count == 1){
        return 0;
    }
    if (count == 2){
        int temp;
        ens.reset(0);
        temp = ens.log();
        ens.set(0);
        return chord_cache[temp];
    }

    if (const auto& it = cache.find(ens); it != cache.end()){
        return it->second;
    }

    double res = INFINITY;
    double v;

    if (!ens.b){
        int beg2 = 0;
        int temp;
        opt_set X(n);
        X.b = true;
        X.i = ens.i;
        X.reset(0);
        temp = X.log();
        beg2 += temp;
        X.rotate(temp);
        while(beg2 < n){
            v = pre_bruteforce_opt(n, X, cache, chord_cache);
            v += chord_cache[beg2];
            if (v < res) res = v;
            X.reset(0);
            temp = X.log();
            X.set(0);
            X.rotate(temp);
            beg2 += temp;
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

    for (int i = 0; i < (1llu << (count-1)); i++){

        c = 0;
        X.i = 0;
        Xb.i = 0;
        for (int j = 1; j < n; j++){
            if (ens[j] == 1){
                if ((i >> c)%2 == 1){
                    X.set(j);
                } else {
                    Xb.set(j);
                }
                c++;
            }
        }

        X.set(0);
        Xb.set(0);

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
    int rbn = n/2+1;
    ens.i = (1llu << (rbn)) - 1;
    ens.b = false; //two robots
    double res = pre_bruteforce_opt(n, ens, cache, chord_cache);
    free(chord_cache);
    return res+1;
}

int main(){
    cout << bruteforce_opt(30) << endl;
    return 0;
}
