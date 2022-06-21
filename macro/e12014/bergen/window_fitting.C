/*For fitting pad traces near window
 *Using equation [p1]/(exp((x-[p0])/[p3])+1)-[p2] 
 *Load using root -l window_fitting.C
 */
void window_fitting()
{
	//need to load data into a histogram object
	//assume hist
	
}

double fitwindow(double *x,double *par){
	double fitval = par[1]/(TMath::Exp((x-par[0])/par[3])+1)-par[2]; 
	return fitval;
}

//-5 and 5 are range. Change these to desired.
//For now put in function where these are inputs
TF1 *func = new TF1("fit",fitwindow,-5,5,4);

//See notepad for best parameters. Likely also function inputs (0-2 at least)
func -> SetParameters(450,100,20,1);
func -> SetParNames("xloc","jump_height","init_height","temp");


