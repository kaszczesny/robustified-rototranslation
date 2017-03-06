#!/usr/bin/env bash

ARTICLE=Szczesny_Twardowski_MSc

./clean.sh

pdflatex -halt-on-error -interaction errorstopmode ${ARTICLE} &&
bibtex ${ARTICLE} &&
pdflatex -halt-on-error -interaction errorstopmode ${ARTICLE} &&
pdflatex -halt-on-error -interaction errorstopmode ${ARTICLE}

exit $?
