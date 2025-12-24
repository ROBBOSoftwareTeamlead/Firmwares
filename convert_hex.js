const fs = require('fs');
const path = require('path');

// Ищем .hex файл в текущей директории
const files = fs.readdirSync(__dirname);
const hexFiles = files.filter(file => file.toLowerCase().endsWith('.hex'));

if (hexFiles.length === 0) {
  console.error('Ошибка: не найден .hex файл в текущей директории');
  process.exit(1);
}

if (hexFiles.length > 1) {
  console.warn('Предупреждение: найдено несколько .hex файлов, будет использован первый:', hexFiles[0]);
  console.warn('Найденные файлы:', hexFiles.join(', '));
}

// Используем первый найденный .hex файл
const hexFile = path.join(__dirname, hexFiles[0]);
console.log('Используется файл:', hexFile);

// Читаем hex-файл
const content = fs.readFileSync(hexFile, 'utf8');

// Разбиваем на строки и фильтруем
const lines = content.split(/\r?\n/)
  .filter(line => line.trim().startsWith(':'))
  .map(line => line.trim());

// Объединяем через точку
const result = lines.join('.');

// Выводим результат
console.log('Длина результата:', result.length);
console.log('Первые 500 символов:');
console.log(result.substring(0, 500));
console.log('\nПоследние 500 символов:');
console.log(result.substring(result.length - 500));

// Сохраняем в файл (используем имя исходного файла)
const baseName = path.basename(hexFiles[0], '.hex');
const outputFile = path.join(__dirname, baseName + '_converted.txt');
fs.writeFileSync(outputFile, result, 'utf8');
console.log(`\nРезультат сохранен в: ${outputFile}`);

