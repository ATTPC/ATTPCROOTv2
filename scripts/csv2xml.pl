#!/usr/bin/perl
#
# script to convert csv to xml

$csvfilename=$ARGV[0];
@tmp=split(/\./,$csvfilename);
$detname=$tmp[0];
$xmlfilename=$detname.".xml";

print("converting ".$csvfilename." to ".$xmlfilename."\n");

open(CSVFILE,"<".$ARGV[0]);
$header=<CSVFILE>; $header=~s/[\r\n]*$//; $header=~s/"//g;
@parameters=<CSVFILE>;
close(CSVFILE); 

@parameter_names=split(/,/,$header);
if($#parameter_names == 0){
    @parameter_names=split(/\s+/,$header);
}
$id=1;

open(XMLFILE,">".$xmlfilename);
print XMLFILE "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<dataroot>\n";

foreach $line (@parameters){
    $line =~ s/[\r\n]*$//;
    $line=~s/"//g;
    print XMLFILE "<$detname>\n";
    print XMLFILE "<ID>$id</ID>\n";
    $cn = 0;
    @parameter_values=split(/,/,$line);
    if($#parameter_values == 0){
	@parameter_values=split(/\s+/,$line);
    }
    foreach $pname (@parameter_names){
	print XMLFILE "<$pname>$parameter_values[$cn]</$pname>\n";
	$cn ++;
    }
    print XMLFILE "</$detname>\n";
    $id ++;
}

print XMLFILE "</dataroot>\n";
close(XMLFILE);
