#!/bin/sh
set -e
cd "$(dirname "$0")"
npx esbuild src/app.js --bundle --format=iife --minify --target=es2020 --outfile=dist/app.js --log-level=warning
node -e "const fs=require('fs');const t=fs.readFileSync('template.html','utf8');const j=fs.readFileSync('dist/app.js','utf8').replace(/<\/script/gi,'<\\\\/script');const i=t.indexOf('/*APP*/');fs.writeFileSync('../index.html',t.slice(0,i)+j+t.slice(i+7));"
ls -la ../index.html
