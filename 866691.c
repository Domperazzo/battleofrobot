//Venturoli Federico 866691

/*questo programma presi in input le posizioni dei robot e l'intervallo di tempo scelto
 dall'utente tramite l'equazione x(t)=x0+vt calcola la posizione successiva, in funzione di
 dove è situato il robot target*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define E 0.1
#define velocità 1.0
#define cost 57.29578 //180/pi per la conversione da radianti a gradi

typedef struct {
    double x;
    double y;
}coordinata_t;

typedef struct{
    double x;
    double y;
} vel;

void inizializzo_double(coordinata_t A, coordinata_t B, coordinata_t C){
    A.x=0.0;
    A.y=0.0;
    B.x=0.0;
    B.y=0.0;
    C.x=0.0;
    C.y=0.0;
}
//calcolo dell'angolo tramite il coefficiente angolare
double angolo_t(coordinata_t A, coordinata_t B){
    double angolo=0;
    double m; //coefficiente angolare
    double x, y;
    x=B.x-A.x;
    y=B.y-A.y;
    m=y/x;
    angolo=atan(m);
    return angolo;
}
//scomposizione delle componenti della velocità lungo gli assi
vel scomposizione_velocità(double angolo){
    vel v;
    v.x=0;
    v.y=0;
    double cos=0, sin=0;
    
    cos=cosf(angolo);
    sin=sinf(angolo);
    
    v.x=velocità*cos;
    v.y=velocità*sin;
    return v;
}


//la coordinata finale sarà la risultante dell'equazione del moto
coordinata_t posizione_spostamento_t(coordinata_t A, coordinata_t B, int tempo){
    double angAB=0;
    coordinata_t A_nuova;
    A_nuova.x=0.0;
    A_nuova.y=0.0;
    vel vA;
    vA.x=0.0;
    vA.y=0.0;

    angAB=angolo_t(A, B);
    vA=scomposizione_velocità(angAB);
    A_nuova.x=(A.x)+((vA.x)*tempo);
    A_nuova.y=(A.y)+((vA.y)*tempo);
    
    return A_nuova;
}

void aggiorna_coordinate(coordinata_t A, coordinata_t B, coordinata_t C, coordinata_t nA, coordinata_t nB, coordinata_t nC){
    A.x=nA.x;
    A.y=nA.y;
    B.x=nB.x;
    B.y=nB.y;
    C.x=nC.x;
    C.y=nC.y;
}


double misura_distanza(coordinata_t A, coordinata_t B){
    double distanza=0.0;
    double x=0.0, y=0.0, x2=0.0, y2=0.0, somma=0.0;
    x=A.x-B.x;
    y=A.y-B.y;
    y2=powf(y, 2);
    x2=powf(x, 2);
    somma=x2+y2;
    distanza=sqrtf(somma);
    return distanza;
}

void controllo_urto(double distanza, int controllo){
    if (distanza<=E) {
        controllo=1;
    } else{
        controllo=0;
    }
}

void bypass(int controllo1, int controllo2, int controllo3, int bypass){
    if (controllo1==1 || controllo2==1 || controllo3==1) {
        bypass=1;
    }
}


void riempi_vettore(int riga, double vettore_coordinate[][6], coordinata_t A, coordinata_t B, coordinata_t C){
    vettore_coordinate[riga][0]=A.x;
    vettore_coordinate[riga][1]=A.y;
    vettore_coordinate[riga][2]=B.x;
    vettore_coordinate[riga][3]=B.y;
    vettore_coordinate[riga][4]=C.x;
    vettore_coordinate[riga][5]=C.y;
    
}

void simulazione_movimento_e_file(coordinata_t A, coordinata_t B, coordinata_t C, int tempo, double vettore_coordinate[tempo][6]){//tempo=k
    coordinata_t nA, nB, nC;
    
    
    double distanzaAB=0.0, distanzaBC=0.0, distanzaCA=0.0;
    int urtoAB=0, urtoBC=0, urtoCA=0, check=0, i=0, riga=0, temp=0, time=1;     /*temp: se ci vuole meno tempo per far scontrare i robot nel                                                                           file ci saranno temp righe*/
    do {
        inizializzo_double(nA, nB, nC);
        
        nA=posizione_spostamento_t(A, B, time);
        nB=posizione_spostamento_t(B, C, time);
        nC=posizione_spostamento_t(C, A, time);
        
        distanzaAB=misura_distanza(nA, nB);
        distanzaBC=misura_distanza(nB, nC);
        distanzaCA=misura_distanza(nC, nA);
        
        controllo_urto(distanzaAB, urtoAB);
        controllo_urto(distanzaBC, urtoBC);
        controllo_urto(distanzaCA, urtoCA);
        
        bypass(urtoAB, urtoBC, urtoCA, check);
        i=i+1;
        time=time+1;
        
        riempi_vettore(riga, vettore_coordinate, nA, nB, nC);
        riga=riga+1;
        aggiorna_coordinate(A, B, C, nA, nB, nC);
        
    } while (i<tempo || check==1);
    
    if (i<tempo) {
        temp=i;
    } else {
        temp=tempo;
    }
    
    FILE *collisioni;
    
    collisioni = fopen("robot.dat", "w");
    
    if(collisioni == NULL){
            printf("Errore\n");
            exit(EXIT_FAILURE);
        }
   
    for (int i=0; i<temp; i=i+1) {
        fprintf(collisioni, "%lf %lf %lf %lf %lf %lf\n", vettore_coordinate[i][0], vettore_coordinate[i][1], vettore_coordinate[i][2], vettore_coordinate[i][3], vettore_coordinate[i][4], vettore_coordinate[i][5]);
    }
    fclose(collisioni);
}

int main(int argc, const char * argv[]) {
    
    int k=0; //numero di unità di tempo da inserire dalla riga di comando
    k = atoi(argv[1]);
    if (argc < 2) {
        printf("Non è stato inserito il numero richiesto.");
        exit(EXIT_FAILURE);
    }
    
    coordinata_t A, B, C;
    inizializzo_double(A, B, C);
    
    double vettore_coordinate[k][6];
    
    scanf("%lf %lf %lf %lf %lf %lf", &A.x, &A.y, &B.x, &B.y, &C.x, &C.y);
    
    simulazione_movimento_e_file(A, B, C, k, vettore_coordinate);

    return 0;
}
