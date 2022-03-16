/* Micro algoritmo genetico para robot movil
Autor: Jose Eduardo Cardoza Plata
Creado el: 10 de Marzo de 2021
Lugar: Ciudad de Mexico, Mexico. */
#include <sys/types.h>
#include<sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include<time.h>

void avanzar(double distancia,double x[],double y[],double z[],double roll[],double pitch[],double yaw[],float time[],int cont[],char aux[],char str[],float xpenal[],float ypenal[],int flag[]);
void girar(double grados,double x[],double y[],double z[],double roll[],double pitch[],double yaw[],float time[],int cont[],char aux[],char str[]);
void EvaluacionAptitud2(int tamPoblacion, int tamCromosoma, int poblacion[][144],int movimientos, float metaX, float metaY,float distR[],float bestD[],int mov[]);
void Cruza(int tamPoblacion,int tamCromosoma,int tamPoblacionCruzada,int poblacion[][144],int poblacionCruzada[][144],int movimientos, float metaX, float metaY, int fragmento, int serie);
void EvaluacionAptitud(int tamPoblacion, int tamCromosoma, int poblacion[][144],int movimientos, float metaX, float metaY,float distR[],int mov[]);
void AutoRegulacion(int tamPoblacion,int tamPoblacionCruzada,int tamCromosoma,int poblacionCruzada[][144],int nuevaPoblacion[][144]);
void Cruzamiento(int tamCromosoma, int hijo[],int padre[],int madre[],int movimientos, float metaX, float metaY, int fragmento, int serie);
float FuncionAptitud(int tamCromosoma, int cromosoma[], int movimientos, float metaX, float metaY,int mov[]);
void PoblacionAleatoria(int tamPoblacion, int tamCromosoma,int poblacionInicial[][144]);
void ImprimePoblacion(int tamPoblacion, int tamCromosoma,int poblacion[][144]);
void colision(float xini,float yini,float xpenal[],float ypenal[],int flag[]);
int Comparador(int tamCromosoma,int CromosomaA[],int CromosomaB[]);
void BinarioAleatorio(int tamCromosoma, int cromosoma[]);
void Suma(int gen, int hijo[],int x,int y);
void trataSenal (int numeroSenhal);
void forky(int mov[], int movTotal);

void forky(int mov[],int movTotal){
	int z = 0;
	if(mov[0] <= 3 ){
		z = 3;
	}
	else if(mov[0] > 3 && mov[0] < (movTotal*2)){
		z = mov[0];
	}
	else if(mov[0] >= (movTotal*2) ){
		z = movTotal*2;
	} //10 es movTotal
	z = z * 2;
	printf("\nmovimientos: %d tiempo: %d seg\n",mov[0],z);
	
	char *cmd[] = { "gzserver","--record","--record_path","prueba4.log","--record_encoding","txt","prueba4.world",(char *)0 };

	/* Identificador del proceso hijo */
	pid_t idProceso;
    
    /*  Mediante esta instruccion se usan las variables actuales 
     * del sistema en la variable environ que se usara despues en execve */
        extern char** environ;
    
	/* Se crea el proceso hijo y se comprueba el error */
	idProceso = fork();

	if (idProceso == -1){
		perror ("No se puede lanzar proceso");
		exit(-1);
	}
	/* Camino que sigue el proceso hijo.
	 * Pone trataSenal() para tratar la señal SIGUSR1 y se mete en un bucle
	 * de espera
	 */
	if (idProceso == 0){
	signal (SIGUSR1, trataSenal);
        //printf("Id Hijo %d\n", getpid());
        /* Unas vez creado el proceso hijo se usa esta instruccion para 
         * cambiar de directorio al directorio de trabajo   */
	chdir("/home/user/Desktop");
        
        /* La llamada a execve pasa el identificador de proceso creado a 
         * el proceso hijo al gzserver para poder administrarlo mediante
         * su ID, es necesario el conjunto de instrucciones cmd y las variables 
         * de entorno environ */
        execve ("/usr/bin/gzserver-9.0.0", cmd, environ);
	}

	/* Camino que sigue el proceso padre.	 */
	if (idProceso > 0){
	printf("\nEsperando gzserver...\n");
       // printf("Id Padre %d\n", getpid());
		sleep (z);
		system("gz log --record=0");
		kill (idProceso, SIGUSR1);
	}
}

void trataSenal (int numeroSenhal){
	printf ("Recibida señal del padre\n");
}

void avanzar(double distancia,double x[],double y[],double z[],double roll[],double pitch[],double yaw[],float time[],int cont[],char aux[],char str[],float xpenal[],float ypenal[],int flag[]){ 
	double x1, y1,xini,yini,a;
	int pasos,i;
	//sin reversa
	if (distancia <= 0){
		 distancia = distancia * -1;
	}
	pasos = round(100 * distancia);
	pasos = abs(pasos);
	xini = 100 * x[0];
	yini = 100 * y[0];
	time[0]=time[0]+0.75;
	x1 = cos(yaw[0]);
	y1 = sin(yaw[0]);
	x[0] = x[0] + (distancia * x1);
	y[0] = y[0] + (distancia * y1);

	strcat(aux,"\n            <waypoint>\n              <time>");
	sprintf(str, "%.2f", time[0]);
	strcat(aux,str);
	strcat(aux,"</time>\n              <pose>");
	sprintf(str,"%.3f %.3f %.0f %.0f %.0f %.3f",x[0],y[0],z[0],roll[0],pitch[0],yaw[0]);
	strcat(aux,str);
	strcat(aux,"</pose>\n            </waypoint>");
	if(time[0] >= 10){
		cont[0]= cont[0] + 134;	
	}
	else if(time[0] < 10){
		cont[0]= cont[0] + 133;
	}
	//comprobacion de colision
	if (distancia >= 0){
		 a = 1;
	}	
	else{
		a = -1;
	}

	for(i=0;i<pasos;i++){
		if(flag[0]==0){
			xini = xini + (a * x1);
			yini = yini + (a * y1);
			colision(xini,yini,xpenal,ypenal,flag);
		}
	}
}

void colision(float xini,float yini,float xpenal[],float ypenal[],int flag[]){

	float i,j,p1yi,p1yf,p1x,p2yi,p2yf,p2x,p3xi,p3xf,p3y,p4xi,p4xf,p4y,obs1yi,obs1yf,obs1x,obs2yi,obs2yf,obs2x;
	
	p1yi = -75;
	p1yf = 75;
	p1x = -50;
	p2yi = -75;
	p2yf = 75;
	p2x = 250;
	p3xi = -50;
	p3xf = 250;
	p3y = 75;
	p4xi = -50;
	p4xf = 250;
	p4y = -75;
	obs1yi = -75;
	obs1yf = 0;
	obs1x = 50;
	obs2yi = 0;
	obs2yf = 75;
	obs2x = 100;

	//pared1
	for(i=(xini-10.581);i<=(xini+10.581);i++){
		for(j=(yini-10.581);j<=(yini+10.581);j++){ //10.581 es el radio del circulo de volumen
				if(i >= (p1x-0.5) && i <= (p1x + 0.5)  ){ //se le suma y resta 0.5 ya que los muros tienen 1 cm de grosor
					if(j >= p1yi && j <= p1yf){
						flag[0] = 1;
						xpenal[0] = xini;
						ypenal[0] = yini;
					}
				}
		}
	}
	//pared2
	if(flag[0]==0){
		for(i=(xini-10.581);i<=(xini+10.581);i++){
			for(j=(yini-10.581);j<=(yini+10.581);j++){
					if(i >= (p2x-0.5) && i <= (p2x + 0.5)  ){ 
						if(j >= p2yi && j <= p2yf){
							flag[0] = 1;
							xpenal[0] = xini;
							ypenal[0] = yini;
						}
					}
			}
		}
	}
	//pared3
	if(flag[0]==0){
		for(i=(xini-10.581);i<=(xini+10.581);i++){
			for(j=(yini-10.581);j<=(yini+10.581);j++){
					if(j >= (p3y-0.5) && j <= (p3y + 0.5)  ){ 
						if(i >= p3xi && i <= p3xf){
							flag[0] = 1;
							xpenal[0] = xini;
							ypenal[0] = yini;
						}
					}
			}
		}
	}
	//pared4
	if(flag[0]==0){
		for(i=(xini-10.581);i<=(xini+10.581);i++){
			for(j=(yini-10.581);j<=(yini+10.581);j++){
					if(j >= (p4y-0.5) && j <= (p4y + 0.5)  ){ 
						if(i >= p4xi && i <= p4xf){
							flag[0] = 1;
							xpenal[0] = xini;
							ypenal[0] = yini;
						}
					}
			}
		}
	}
	//obstaculo 1
	if(flag[0]==0){
		for(i=(xini-10.581);i<=(xini+10.581);i++){
			for(j=(yini-10.581);j<=(yini+10.581);j++){
					if(i >= (obs1x-0.5) && i <= (obs1x + 0.5)  ){ 
						if(j >= obs1yi && j <= obs1yf){
							flag[0] = 1;
							xpenal[0] = xini;
							ypenal[0] = yini;
						}
					}
			}
		}
	}
	//obstaculo 2
	if(flag[0]==0){
		for(i=(xini-10.581);i<=(xini+10.581);i++){
			for(j=(yini-10.581);j<=(yini+10.581);j++){
					if(i >= (obs2x-0.5) && i <= (obs2x + 0.5)  ){ 
						if(j >= obs2yi && j <= obs2yf){
							flag[0] = 1;
							xpenal[0] = xini;
							ypenal[0] = yini;
						}
					}
			}
		}
	}


}

void girar(double grados,double x[],double y[],double z[],double roll[],double pitch[],double yaw[],float time[],int cont[],char aux[],char str[]){
	double x1, y1,xini,yini;
	xini = x[0];
	yini = y[0];
	time[0]=time[0]+0.75;
	if(grados > 0){
		x1 = yaw[0] + grados;
		if(x1 > 6.283){
			y1 = x1 - 6.283;
			x1 = y1;
		}
	}
	if(grados < 0){
		x1 = yaw[0] + grados;
		if(x1 < -6.283){
			y1 = x1 + 6.283;
			x1 = y1;
		}
	}
	yaw[0] = x1;

	strcat(aux,"\n            <waypoint>\n              <time>");
	sprintf(str, "%.2f", time[0]);
	strcat(aux,str);
	strcat(aux,"</time>\n              <pose>");
	sprintf(str,"%.3f %.3f %.0f %.0f %.0f %.3f",x[0],y[0],z[0],roll[0],pitch[0],yaw[0]);
	strcat(aux,str);
	strcat(aux,"</pose>\n            </waypoint>");
	if(time[0] >= 10){
		cont[0]= cont[0] + 134;	
	}
	else if(time[0] < 10){
		cont[0]= cont[0] + 133;
	}
}

//--------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------


//funcion que genera pseudoaleatoriamente cada bit de un Cromosoma en representacion binaria
void BinarioAleatorio(int tamCromosoma, int cromosoma[]){
	int i,numeroAleatorio;

	for(i=0;i<tamCromosoma;i++){
		numeroAleatorio = rand()%100;
		if(numeroAleatorio <= 49){
			cromosoma[i] = 0;
		}
		else{
			cromosoma[i] = 1;
		}
	}
}

//Funcion que genera la poblacion inicial, generando cada bit de cada Cromosoma pseudoaleatoriamente
void PoblacionAleatoria(int tamPoblacion, int tamCromosoma,int poblacionInicial[][144]){
	int i;
	srand(time(NULL));

	for(i=0;i<tamPoblacion;i++){
		BinarioAleatorio(tamCromosoma,poblacionInicial[i]);
	}
}

//funcion para imprimir en consola los Cromosomas de una poblacion
void ImprimePoblacion(int tamPoblacion, int tamCromosoma,int poblacion[][144]){
	int i,j;
	for(i=0;i<tamPoblacion;i++){
		printf("\nCromosoma %i:",i+1);
		for(j=0;j<tamCromosoma;j++){
			printf(" %i",poblacion[i][j]);
		}
	}
}

//Funcion que determina la aptitud de un Cromosoma*****************************************************************************************
float FuncionAptitud(int tamCromosoma, int cromosoma[], int movimientos, float metaX, float metaY,int mov[]){
	char buffer[8192] = "", buffer2[1998192]="", aux[1998192]="";
	char Ruta[30]="", RutaSal[30]="", RutaRes[30]="", dato;
	//char x = '8';
	FILE *Archivo, *Salida, *Resultado;
	float pos,aptitud,penalizacion,grados,distancia,time[1],xpenal[1],ypenal[1];
	char str[1998192]="",espacio;
	int i,j,k,l,m,giro,avance,tamBuffer,tamCad,cont[1],psl,flag[1];
	double x[1],y[1],z[1],roll[1],pitch[1],yaw[1],posFinal[6]={0};

	i=j=k=l=m=tamBuffer=tamCad=cont[0]=time[0]=psl=flag[0]= 0;
	x[0]=y[0]=z[0]=roll[0]=pitch[0]=yaw[0]=time[0]=xpenal[0]=ypenal[0]= 0;
	mov[0] = 0;
	aptitud = penalizacion = pos = giro = grados = distancia = 0;
	avance = 14;

	strcat(Ruta, "prueba2.world");
	strcat(RutaSal,"prueba4.world");

	//apertura de archivo de entrada
	if((Archivo = fopen(Ruta, "r")) == NULL){perror(Ruta); exit (-1);}
	i=0;
	while(!feof(Archivo)){
		fscanf(Archivo, "%c", &dato);
		buffer[i] = dato;
		i++;
	}
	fclose(Archivo);

	//Anadimos los movimientos deseados al archivo world desde el caracter 1890
	k=6030;
	cont[0] = k;
	for(j=0;j<k+1;j++){
		aux[j] = buffer[j];
	}
	//primero verificamos si se avanza/gira o no, se recorre la parte del cromosoma de la distancia/radianes, se convierte de binario a decimal y se ejecuta el movimiento
	for(j=0;j<movimientos;j++){
		//if(cromosoma[giro] == 1){
			m = 0;
			for(l=giro;l<=(giro+13);l++){
				if(cromosoma[l] == 1){
					grados = grados + pow(2,m);
				}
				m = m + 1;
			}
			grados =  -6.284 + (grados*((6.284 -(-6.284))/(pow(2,14)-1)));
			girar(grados,x,y,z,roll,pitch,yaw,time,cont,aux,str);
			mov[0] = mov[0] + 1;
		//}
		//if(cromosoma[avance] == 1){
			m = 0;
			for(l=avance;l<=(avance+9);l++){
				if(cromosoma[l] == 1){
					distancia = distancia + pow(2,m);
				}
				m = m + 1;
			}
			distancia =  -0.5 + (distancia*((0.5 -(-0.5))/(pow(2,10)-1)));
			avanzar(distancia,x,y,z,roll,pitch,yaw,time,cont,aux,str,xpenal,ypenal,flag);
			mov[0] = mov[0] + 1;
		//}
		giro = giro + 24;
		avance = avance + 24;
	}
	//al final guardamos el tamano de la cadena aux, es la que se anade, y el numero de movimientos realizados.
	tamCad = strlen(aux);

	//se anadieron tamCad caracteres
	for(j=k;j<i;j++){
		aux[tamCad] = buffer[j];
		tamCad++;
	}
	//Archivo de salida resultante prueba4.world
	if((Salida = fopen(RutaSal, "w")) == NULL){perror(RutaSal); exit (-1);}
	j=0;
	while(j < tamCad){

		fprintf(Salida, "%c", aux[j]);
		j++;
	}
	fclose(Salida);		

	/*for(j=0;j<tamCad;j++){
		printf("%c", aux[j]);
	}*/
	//printf("\nEsperando gzserver...\n");
	forky(mov, movimientos);
	//system("gzserver -r --record_path prueba4.log --record_encoding txt prueba4.world");
	printf("\nGzserver ha terminado satisfactoriamente...");
	//system("killall -9 gzserver");
	system("gz log -e -f prueba4.log/state.log -z 30 --filter *.pose/*.pose > filtered_state2.log");
	strcat(RutaRes,"filtered_state2.log");

	//Leer el archivo log filtrado
	if((Resultado = fopen(RutaRes, "r")) == NULL){perror(RutaRes); exit (-1);}
	k=0;
	while(!feof(Resultado)){
		fscanf(Resultado, "%c", &dato);
		buffer2[k] = dato;
		k++;
	}
	fclose(Resultado);
	for(j=(k-3);j>0;j--){
		if(buffer2[j] == 't' && buffer2[j+1] == 'u' &&buffer2[j+2] == 'k'){
			psl = j+12;
			j=0;
		}
	}
	if((Resultado = fopen(RutaRes, "r")) == NULL){perror(RutaRes); exit (-1);}
	for(j=0;j<=k;j++){
		fscanf(Resultado, "%c", &dato);
		espacio = dato;
		if(j == (psl-1)){
			fscanf(Resultado, "%f", &pos);
			posFinal[0] = pos;
			fscanf(Resultado, "%c", &dato);
			espacio = dato;
			fscanf(Resultado, "%f", &pos);
			posFinal[1] = pos;
			fscanf(Resultado, "%c", &dato);
			espacio = dato;
			fscanf(Resultado, "%f", &pos);
			posFinal[2] = pos;
			fscanf(Resultado, "%c", &dato);
			espacio = dato;
			fscanf(Resultado, "%f", &pos);
			posFinal[3] = pos;
			fscanf(Resultado, "%c", &dato);
			espacio = dato;
			fscanf(Resultado, "%f", &pos);
			posFinal[4] = pos;
			fscanf(Resultado, "%c", &dato);
			espacio = dato;
			fscanf(Resultado, "%f", &pos);
			posFinal[5] = pos;
			fscanf(Resultado, "%c", &dato);
			espacio = dato;	
		}
	}
	fclose(Resultado);

	printf("\nposicion final: \n");
 	printf("posX: %.3f; posY: %.3f",posFinal[0],posFinal[1]);
	printf("\nmetaX: %.3f; metaY: %.3f\n",metaX,metaY);
	aptitud = sqrt(pow((metaX - posFinal[0]),2) + pow((metaY - posFinal[1]),2));
	printf("distancia o aptitud : %.3f\n",aptitud);
	if(flag[0]==1){
		printf("colision en X: %.3f; Y: %.3f\n",xpenal[0]/100,ypenal[0]/100);
		penalizacion = sqrt(pow((metaX - (xpenal[0]/100)),2) + pow((metaY - (ypenal[0]/100)),2));//PENALIZA = SOLO LA POSICION DE COLISION y la meta 
		printf("penalizacion: %.3f\n",penalizacion); 
		aptitud = penalizacion+(0.10581*3);//Aumentamos penalizacion al colisionar, multiplicamos por 5 ya que haci no lo regresamos a enfrente del muro, sino dos tamanho de robot atras
		printf("aptitud final: %.3f\n",aptitud); //Dandole chanse a soluciones si colisionar tengan buena aptitud, y no solo 1 y solo 1 que era la solucion antes de colisionar 
	}
	return aptitud;
}

/*Funcion que determina la aptitud de los Cromosomas de la Poblacion Inicial guardandola en un arreglo, el cual, se ordenara utilizando
el ordenamiento burbuja para, posteriormente, ordenar dicha Poblacion Inicial por aptitud */
void EvaluacionAptitud(int tamPoblacion, int tamCromosoma, int poblacion[][144],int movimientos, float metaX, float metaY,float distR[],int mov[]){
	float aptitud,aux,aux2;
	int i,j,k;
	float poblacionOrdenada[tamPoblacion][2];
	int aux3[tamPoblacion][tamCromosoma];

	for(i=0;i<tamPoblacion;i++){
		printf("\nevaluando cromosoma: %d \n", i+1);
		aptitud = FuncionAptitud(tamCromosoma,poblacion[i],movimientos,metaX,metaY,mov);
		poblacionOrdenada[i][0]= aptitud;
		poblacionOrdenada[i][1]= i;
	}

	//Ordenamiento Burbuja
	for(i=1;i<tamPoblacion;i++) {
		for(j=0;j<(tamPoblacion-i);j++) {
			if(poblacionOrdenada[j][0] > poblacionOrdenada[j+1][0]){
				aux = poblacionOrdenada[j][0];
		        poblacionOrdenada[j][0] = poblacionOrdenada[j+1][0];
		        poblacionOrdenada[j+1][0] = aux;

		        aux2 = poblacionOrdenada[j][1];
		        poblacionOrdenada[j][1] = poblacionOrdenada[j+1][1];
		        poblacionOrdenada[j+1][1] = aux2;
			}
		}
	}

	//ordenamiento de la poblacion inicial deacuerdo a su aptitud
	for(i=0;i<tamPoblacion;i++){
		k = poblacionOrdenada[i][1]; //posicion del Cromosoma ordenado de menor a mayor aptitud
		for(j=0;j<tamCromosoma;j++){
			aux3[i][j] = poblacion[k][j]; //guardamos en un arreglo auxiliar los Cromosomas de la Poblacion Inicial de menor a mayor aptitud
		}
	}
	for(i=0;i<tamPoblacion;i++){ //pasamos la poblacion ya ordenada al arreglo inicial de mayor a menor aptitud
		for(j=0;j<tamCromosoma;j++){
			poblacion[tamPoblacion - i - 1][j] = aux3[i][j];
		}
		distR[tamPoblacion - i - 1] = poblacionOrdenada[i][0];
	}
}

void EvaluacionAptitud2(int tamPoblacion, int tamCromosoma, int poblacion[][144],int movimientos, float metaX, float metaY,float distR[],float bestD[],int mov[]){
	float aptitud,aux,aux2;
	int i,j,k;
	float poblacionOrdenada[tamPoblacion][2];
	int aux3[tamPoblacion][tamCromosoma];

	for(i=0;i<tamPoblacion;i++){
		printf("\nevaluando cromosoma: %d \n", i+1);
		if(i==0){
			aptitud = distR[0];
			printf("\ndistancia del padre 1: %.3f \n", distR[0]);
		}
		else if(i==1){
			aptitud = distR[1];
			printf("\ndistancia del padre 2: %.3f \n", distR[1]);
		}
		else if(i==3){
			aptitud = distR[2];
			printf("\ndistancia del padre 3: %.3f \n", distR[2]);
		}
		else if(i==6){
			aptitud = distR[3];
			printf("\ndistancia del padre 4: %.3f \n", distR[3]);
		}
		else if(i==10){
			aptitud = distR[4];
			printf("\ndistancia del padre 5: %.3f \n", distR[4]);
		}
		else {
			aptitud = FuncionAptitud(tamCromosoma,poblacion[i],movimientos,metaX,metaY,mov);
		}
		poblacionOrdenada[i][0]= aptitud;
		poblacionOrdenada[i][1]= i;
	}

	//Ordenamiento Burbuja
	for(i=1;i<tamPoblacion;i++) {
		for(j=0;j<(tamPoblacion-i);j++) {
			if(poblacionOrdenada[j][0] > poblacionOrdenada[j+1][0]){
				aux = poblacionOrdenada[j][0];
		        poblacionOrdenada[j][0] = poblacionOrdenada[j+1][0];
		        poblacionOrdenada[j+1][0] = aux;

		        aux2 = poblacionOrdenada[j][1];
		        poblacionOrdenada[j][1] = poblacionOrdenada[j+1][1];
		        poblacionOrdenada[j+1][1] = aux2;
			}
		}
	}

	//ordenamiento de la poblacion inicial deacuerdo a su aptitud
	for(i=0;i<tamPoblacion;i++){
		k = poblacionOrdenada[i][1]; //posicion del Cromosoma ordenado de menor a mayor aptitud
		for(j=0;j<tamCromosoma;j++){
			aux3[i][j] = poblacion[k][j]; //guardamos en un arreglo auxiliar los Cromosomas de la Poblacion Inicial de menor a mayor aptitud
		}
	}
	for(i=0;i<tamPoblacion;i++){ //pasamos la poblacion ya ordenada al arreglo inicial de mayor a menor aptitud
		for(j=0;j<tamCromosoma;j++){
			poblacion[tamPoblacion - i - 1][j] = aux3[i][j];
		}
		distR[tamPoblacion - i - 1] = poblacionOrdenada[i][0];
	}
	bestD[0] = poblacionOrdenada[0][0];
}


/* Funcion de cruza controlada donde el mejor Cromosoma se cruza con toda la poblacion, teniendo genes dominantes, el segundo mas apto Cromosoma se cruza
 * con el resto de la poblacion y asi sucesivamente hasta llegar al ultimo Cromosoma el cual no se cruza con nadie como padre dominante.
 * La poblacion resultante la integran los padres y todos los hijos, por ejemplo, en este caso, con 5 Cromosomas tendremos 10 hijos, 4 del mejor Cromosoma
 * como padre dominante, 3 del segundo mejor, 2 del tercero mas apto, 1 del cuarto mas apto y ninguno del menos apto como padre dominante. Garantizando
 * con este procedimeinto que cada Cromosoma tenga 5 hijos heredando asi su conociemto del espacio de busqueda explorado en medida de su funcion Aptitud.
 */
void Cruza(int tamPoblacion,int tamCromosoma,int tamPoblacionCruzada,int poblacion[][144],int poblacionCruzada[][144],int movimientos, float metaX, float metaY, int fragmento, int serie){
	int i,j,k,cont,numHijos[tamCromosoma],hijo[tamCromosoma];
	cont=0;

		for(i=0;i<tamPoblacion;i++){
			numHijos[tamPoblacion -i - 1]= tamPoblacion - i - 1; //Determinamos el numero de hijos en funcion de la aptitud de cada Cromosoma
		}

		for(i=0;i<tamPoblacion;i++){
			//printf("\n\nCromosoma %d, padre de %d hijos\n",i+1,numHijos[i]);
			for(j=0;j<(1 + numHijos[i]);j++){
				if (j != 0){
				printf("\nCruzamiento %d, el padre es %d , %d es la madre\n",cont+1,i+1,j);
			//	printf("\nel inidivduo %d es la madre\n",j);
					Cruzamiento(tamCromosoma,hijo,poblacion[i],poblacion[j-1],movimientos,metaX,metaY,fragmento,serie); //Funcion de cruzamiento desde el mas apto hasta el peor

				}
				for(k=0;k<tamCromosoma;k++){
					if(j == 0){
						poblacionCruzada[cont][k]=poblacion[i][k]; //el padre integra a la nueva poblacion
					}
					else{
						poblacionCruzada[cont][k] = hijo[k]; //el hijo integra la nueva poblacion
					}
				}
				cont= cont + 1;
			}
		}
}

//Funcion de cruzamiento de dos Cromosomas que genera un hijo
void Cruzamiento(int tamCromosoma, int hijo[],int padre[],int madre[],int movimientos, float metaX, float metaY, int fragmento, int serie){
	int i,j,gen,operacion,x,aux,numAleatorio,gen2,fragmento2;
	float pMutacion;
	fragmento2 = 24 * (rand()%6);

	//aptitudPadre = FuncionAptitud(tamCromosoma,padre,movimientos,metaX,metaY);

	//printf("hijo original...");
	for(i=0;i<tamCromosoma;i++){
		hijo[i] = padre [i]; //copiamos el Cromosoma padre en el hijo
	//	printf(" %d",hijo[i]);
	}
	printf("\ncruzando fragmento %d",1+(fragmento/24));
	for(i=0;i<(serie);i++){ //cambiamos n genes del hijo por los genes de la madre, de aceurdo a la difrencia entreel tamano y la aptitud del padre
		gen = rand()%24;
		gen = gen + fragmento;
		
		//operacion = rand()%4;
		//if (operacion == 0){//Conservar
		//	hijo[gen] = padre[gen];
		//}
		//if (operacion == 1){//Restar
		//	hijo[gen] = (int)fabs(hijo[gen]-madre[gen]);
		//}
		//else if (operacion == 2){//Sumar
		//	x = padre[gen] + madre[gen];
		//	if(gen == 0){ //caso de ser el ultimo gen para evitar desbordamiento del arreglo
		//		hijo[gen] = x%2;
		//	}
		//	else{
		//		Suma(gen,hijo,padre[gen],madre[gen]);
		//	}
		//}
		//else if (operacion == 3){//Sustituir
			hijo[gen] = madre[gen];
		//}

	}

	//SE INCORPORO UN SEGUNDO FRAGMENTO PARA QUE CRUZE EL PRIMER Y SEGUNDO MOVIVIENTO AVANZANDO HASTA EL 4 Y 5TO, FINALMENTE SOLO CRUZA EL 5TO MOVIMIENTO
	//if(fragmento2 <= 134){
		printf("\ncruzando movimeinto 2: %d",1+(fragmento2/24));
		for(i=0;i<(serie);i++){ //cambiamos n genes del hijo por los genes de la madre, de aceurdo a la difrencia entreel tamano y la aptitud del padre
			gen2 = rand()%24;
			gen2 = gen2 + fragmento2;
		
		//	operacion = rand()%4;
		//	if (operacion == 0){//Conservar
		//		hijo[gen2] = padre[gen2];
		//	}
		//	if (operacion == 1){//Restar
		//		hijo[gen2] = (int)fabs(hijo[gen2]-madre[gen2]);
		//	}
		//	else if (operacion == 2){//Sumar
		//		x = padre[gen2] + madre[gen2];
		//		if(gen2 == 0){ //caso de ser el ultimo gen para evitar desbordamiento del arreglo
		//			hijo[gen2] = x%2;
		//		}
		//		else{
		//			Suma(gen2,hijo,padre[gen2],madre[gen2]);
		//		}
		//	}
		//	else if (operacion == 3){//Sustituir
				hijo[gen2] = madre[gen2];
		//	}
		}	
	//}
	//else{
	//	printf("\nsolo se cruza 1 movimiento");
	//}


	//Mutacion
	pMutacion = 0; //PROBABILIDAD MUTACION 1/144 BITS
	for(i=0;i<tamCromosoma;i++){
		aux = rand()%144;
		//numAleatorio = ((float)aux)/100;
		if(aux <= pMutacion){
			if(hijo[i] == 0){
				hijo[i] = 1;
			}
			else if(hijo[i] == 1){
				hijo[i] = 0;
			}
		}
	}
}

//funcion que realiza la operacion suma, en caso de que exista acarreo
void Suma(int gen, int hijo[],int x,int y){
	int valor = x + y;
	hijo[gen] = valor%2;
	valor = valor/2;
	if(gen != 0){
		Suma(gen-1,hijo,valor,hijo[gen-1]);
	}
}

/*Funcion que regula la poblacion, seleccionado los 2 cromosomas mas aptos y 3 aleatoriamente, regresando asi a su tamaño original*/
void AutoRegulacion(int tamPoblacion,int tamPoblacionCruzada,int tamCromosoma,int poblacionCruzada[][144],int nuevaPoblacion[][144]){
	int seguro,bandera,i,j,x,y,z,igualdad1,igualdad2,igualdad3,igualdad4;
	//int x,y,z,j;
	//x=y=z=0;
	seguro=bandera=0;
	i=2;

	/*Seleccionamos 3 individuos aleatoriamente de la poblacion clonada, sin importar si los individuos son iguales entre si*/
/*	//while(y==x||y==z||z==x){
		x=rand()%(tamPoblacionCruzada-1);
		y=rand()%(tamPoblacionCruzada-1);
		z=rand()%(tamPoblacionCruzada-1);
	//}

	for(j=0;j<tamCromosoma;j++){
		nuevaPoblacion[0][j]=poblacionCruzada[x][j];
		nuevaPoblacion[1][j]=poblacionCruzada[y][j];
		nuevaPoblacion[2][j]=poblacionCruzada[z][j];
		nuevaPoblacion[3][j]=poblacionCruzada[tamPoblacionCruzada-2][j];//Elitismo del segundo mejor clon
		nuevaPoblacion[4][j]=poblacionCruzada[tamPoblacionCruzada-1][j];//Elitismo del mejor clon
	}*/



/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

	for(j=0;j<tamCromosoma;j++){
		nuevaPoblacion[tamPoblacion-1][j]=poblacionCruzada[tamPoblacionCruzada-1][j];//Elitismo del mejor cromosoma
	}
	while(i<=tamPoblacionCruzada){
		igualdad1 = Comparador(tamCromosoma,poblacionCruzada[tamPoblacionCruzada-1],poblacionCruzada[tamPoblacionCruzada-i]);
		if(igualdad1 == 0){
			for(j=0;j<tamCromosoma;j++){
				nuevaPoblacion[tamPoblacion-2][j]=poblacionCruzada[tamPoblacionCruzada-i][j];//Elitismo del segundo mejor cromosoma
				}
			seguro = 1;
			i = tamPoblacionCruzada + 1;
		}
		i = i+1;
	}
	if (seguro == 0){
			for(j=0;j<tamCromosoma;j++){
				nuevaPoblacion[tamPoblacion-2][j]=poblacionCruzada[tamPoblacionCruzada-2][j];
			}
		}

	//Seleccionamos 3 diferentes Cromosomas aleatoriamente
	seguro = 0;
	while(bandera <= (tamPoblacionCruzada*10)){
		x=rand()%(tamPoblacionCruzada-2);
		igualdad1 = Comparador(tamCromosoma,poblacionCruzada[x],nuevaPoblacion[tamPoblacion-1]);
		igualdad2 = Comparador(tamCromosoma,poblacionCruzada[x],nuevaPoblacion[tamPoblacion-2]);
		if(igualdad1 == 0 && igualdad2 == 0){
			bandera = (tamPoblacionCruzada*10)+1;
			seguro = 1;
			for(j=0;j<tamCromosoma;j++){
				nuevaPoblacion[tamPoblacion-3][j]=poblacionCruzada[x][j];//Cromosoma seleccionado aleatoriamente
			}
		}
		bandera = bandera + 1;
	}
	if (seguro == 0){
			for(j=0;j<tamCromosoma;j++){
				nuevaPoblacion[tamPoblacion-3][j]=poblacionCruzada[x][j];
			}
		}
	seguro = 0;
	bandera = 0;
	while(bandera <= (tamPoblacionCruzada*10)){
			y=rand()%(tamPoblacionCruzada-2);
			igualdad1 = Comparador(tamCromosoma,poblacionCruzada[y],nuevaPoblacion[tamPoblacion-1]);
			igualdad2 = Comparador(tamCromosoma,poblacionCruzada[y],nuevaPoblacion[tamPoblacion-2]);
			igualdad3 = Comparador(tamCromosoma,poblacionCruzada[y],nuevaPoblacion[tamPoblacion-3]);
			if(igualdad1 == 0 && igualdad2 == 0 && igualdad3 == 0){
				bandera = (tamPoblacionCruzada*10)+1;
				seguro = 1;
				for(j=0;j<tamCromosoma;j++){
					nuevaPoblacion[tamPoblacion-4][j]=poblacionCruzada[y][j];//Cromosoma seleccionado aleatoriamente
				}
			}
			bandera = bandera + 1;
		}
	if (seguro == 0){
			for(j=0;j<tamCromosoma;j++){
				nuevaPoblacion[tamPoblacion-4][j]=poblacionCruzada[y][j];
			}
		}
	seguro = 0;
	bandera = 0;
	while(bandera <= (tamPoblacionCruzada*10)){
			z=rand()%(tamPoblacionCruzada-2);
			igualdad1 = Comparador(tamCromosoma,poblacionCruzada[z],nuevaPoblacion[tamPoblacion-1]);
			igualdad2 = Comparador(tamCromosoma,poblacionCruzada[z],nuevaPoblacion[tamPoblacion-2]);
			igualdad3 = Comparador(tamCromosoma,poblacionCruzada[z],nuevaPoblacion[tamPoblacion-3]);
			igualdad4 = Comparador(tamCromosoma,poblacionCruzada[z],nuevaPoblacion[tamPoblacion-4]);
			if(igualdad1 == 0 && igualdad2 == 0 && igualdad3 == 0 && igualdad4 == 0){
				bandera = (tamPoblacionCruzada*10)+1;
				seguro = 1;
				for(j=0;j<tamCromosoma;j++){
					nuevaPoblacion[tamPoblacion-5][j]=poblacionCruzada[z][j];//Cromosoma seleccionado aleatoriamente
				}
			}
			bandera = bandera + 1;
		}
	if (seguro == 0){
			for(j=0;j<tamCromosoma;j++){
				nuevaPoblacion[tamPoblacion-5][j]=poblacionCruzada[z][j];
			}
		}

}

int Comparador(int tamCromosoma,int CromosomaA[],int CromosomaB[]){
	int igualdad,i,contador;
	contador=0;

	for(i=0;i<tamCromosoma;i++){
		if(CromosomaA[i]==CromosomaB[i]) contador = contador +1;
	}

	if(contador==tamCromosoma) igualdad = 1;
	else igualdad = 0;

	return igualdad;
}
//----------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------


int main(void){
	int g,h,i,j,k,w,e,fragmento,rfrag,serie,flag,movimientos,tamCromosoma,condicionParada,convergenciaNominal,tamPoblacion,tamPoblacionCruzada,mov[1],poblacionCruzada[15][144],nuevaPoblacion[5][144];
	int cromosoma[144];
	int poblacionInicial[5][144];
	float afinidad,metaX,metaY,distR[15],bestD[1];
	struct timeval tinicio, tfin;
	float secsFinal,t_sec,t_usec;
	char RutaGrafica[50]="";
	char RutaMundo[50]="";
	char strGrafica[10000]="";
   	char auxGrafica[10000]="";
	char RutaEntrada[50]="";
   	char auxEntrada[10000]="";
	char datoEntrada;
	strcat(RutaGrafica,"resultadosPrueba1.txt");
	strcat(RutaEntrada,"resultadosPrueba1.txt");
	strcat(RutaMundo,"mejorCromosoma.txt");
	FILE *SalidaGrafica,*ArchivoEntrada, *SalidaMundo;
	
	flag = -1;
	fragmento = 0;
	mov[0] = 0;
	movimientos = 6;
	metaX = 2;
	metaY = 0;
	tamPoblacion = 5;
	tamPoblacionCruzada = 15;
	tamCromosoma = 144;
	condicionParada = 30;
	convergenciaNominal = 10;
	gettimeofday(&tinicio,NULL);

	printf("Generando poblacion inicial...");
	PoblacionAleatoria(tamPoblacion,tamCromosoma,poblacionInicial);

	for(h=0;h<condicionParada;h++){
		//flag = flag + 1;
		//if(flag == 6){
		//rfrag = rand()%5;
		//fragmento = 27 * rfrag;
		//	flag = 0;
		//}
		if(h!=0)printf("\nReinicializacion del algoritmo\n");
		serie = 0;
		for(i=0;i<convergenciaNominal;i++){
			rfrag = rand()%6; //num de movimientos
			fragmento = 24 * rfrag;
			if ((h+1) % 2 == 0){
				serie = 6 + i;
			}
			else{
				serie = 23 - i;
			}
			srand(time(NULL)); // establecemos la semilla en cada reinicio de poblacion
			printf("\n\ngeneracion %d de %d , convergencia %d de %d\n",i+1,convergenciaNominal,h+1,condicionParada);
			printf("\nPoblacion inicial\n");
			//ImprimePoblacion(tamPoblacion,tamCromosoma,poblacionInicial);
			EvaluacionAptitud(tamPoblacion, tamCromosoma, poblacionInicial,movimientos,metaX,metaY,distR,mov);
			printf("\n\npoblacion ordenada por aptitud de mayor a menor\n");
			//ImprimePoblacion(tamPoblacion,tamCromosoma,poblacionInicial);
			for(w=0;w<5;w++){
				printf("\ndistancia del cromosoma %d: %f\n",w,distR[w]);
			}
			printf("\n\nCruzamiento...");
			Cruza(tamPoblacion,tamCromosoma,tamPoblacionCruzada,poblacionInicial,poblacionCruzada,movimientos,metaX,metaY,fragmento,serie);
			printf("\n\nPoblacion cruzada \n");
			//ImprimePoblacion(tamPoblacionCruzada,tamCromosoma,poblacionCruzada);
			printf("\n\nReevaluacion por aptitud\n");
			EvaluacionAptitud2(tamPoblacionCruzada, tamCromosoma, poblacionCruzada,movimientos,metaX,metaY,distR,bestD,mov);
			printf("\npoblacion ordenada por aptitud de mayor a menor\n");
			//ImprimePoblacion(tamPoblacionCruzada,tamCromosoma,poblacionCruzada);
			for(w=0;w<15;w++){
				printf("\ndistancia de cromosoma %d: %f\n",w,distR[w]);
			}
			printf("\n\nElitismo...");
			printf("\n\nNueva Poblacion: 2 mejores cromosomas (seleccion natural) y 3 al azar (deriva genetica)\n");
			AutoRegulacion(tamPoblacion,tamPoblacionCruzada,tamCromosoma,poblacionCruzada,nuevaPoblacion);
			//ImprimePoblacion(tamPoblacion,tamCromosoma,nuevaPoblacion);
			//Si no se ha alcanzado la convergencia nominal, la nueva poblacion sera ahora la poblacion inicial
			for(j=0;j<tamPoblacion;j++){
				for(k=0;k<tamCromosoma;k++){
					poblacionInicial[j][k]=nuevaPoblacion[j][k];
				}
			}
		}

		printf("\n\nConvergencia nominal alcanzada\n");
		printf("\nEvaluando condincion de parada...%d de %d\n",h+1,condicionParada);
		BinarioAleatorio(tamCromosoma,poblacionInicial[0]);
		BinarioAleatorio(tamCromosoma,poblacionInicial[1]);
		BinarioAleatorio(tamCromosoma,poblacionInicial[2]);
		//guardamos la mejor distancia de la convergencia nominal en el archivo de resutados.txt
		//apertura de archivo de resultados
		if((ArchivoEntrada = fopen(RutaEntrada, "r")) == NULL){perror(RutaEntrada); exit (-1);}
		g=0;
		while(!feof(ArchivoEntrada)){
			fscanf(ArchivoEntrada, "%c", &datoEntrada);
			auxEntrada[g] = datoEntrada;
			g++;
		}
		fclose(ArchivoEntrada);
		//escritura del archivo de resultados, anadiendo el nuevo resultado
		if((SalidaGrafica = fopen(RutaGrafica, "w")) == NULL){perror(RutaGrafica); exit (-1);}
		e=0;
		while(e < g){
			fprintf(SalidaGrafica, "%c", auxEntrada[e]);
			e++;
		}
		fprintf(SalidaGrafica, "\n%f", distR[14]);
		fclose(SalidaGrafica);

		//guardamos el mejor cromosoma en el archivo, MejorCromosoma
		if((SalidaMundo = fopen(RutaMundo, "w")) == NULL){perror(RutaMundo); exit (-1);}
		for(i=0;i<tamCromosoma;i++){
			fprintf(SalidaMundo, " %d,",poblacionInicial[tamPoblacion-1][i]);
		}
		fclose(SalidaMundo);
		


	}


	printf("\n\nCondicion de parada alcanzada\n");
	printf("\nConvergencia general terminada\n");
	printf("\n\nResultados\n");
	printf("\nMejor Cromosoma o solucion encontrada\n");
	for(i=0;i<tamCromosoma;i++){
		printf(" %d,",poblacionInicial[tamPoblacion-1][i]);
		//printf(" %d",poblacionInicial[0][i]);
	}
	//afinidad = FuncionAptitud(tamCromosoma,poblacionInicial[0],movimientos,metaX,metaY);
	afinidad = FuncionAptitud(tamCromosoma,poblacionInicial[tamPoblacion-1],movimientos,metaX,metaY,mov);
	//afinidad = bestD[0];
	printf("\n\nMejor distancia encontrada: %f",afinidad);
	printf("\nGeneraciones: %d\n",convergenciaNominal*condicionParada);
	
		
	gettimeofday(&tfin,NULL);
	t_sec = (float) (tfin.tv_sec - tinicio.tv_sec);
	t_usec = (float) (tfin.tv_usec - tinicio.tv_usec);
	secsFinal = t_sec + t_usec / 1.0e+6;
	printf("Tiempo total: %.3f segundos\n\n",secsFinal);

	return 0;
}
