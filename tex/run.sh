#!/usr/bin/env bash

ARTICLE=Szczesny_Twardowski_MSc

./clean.sh

pdflatex -halt-on-error -interaction nonstopmode ${ARTICLE} &&
biber ${ARTICLE} &&
pdflatex -halt-on-error -interaction nonstopmode ${ARTICLE} &&
pdflatex -halt-on-error -interaction nonstopmode ${ARTICLE}

exit $?
