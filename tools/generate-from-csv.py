import csv

# Path to your CSV file
csv_file_paths = ["rawcsv/CAN IDs (comprehensive) - inverter output.csv", "rawcsv/CAN IDs (comprehensive) - inverter input.csv"]
mode = "input"  # or "input"

# Template with placeholders
template = '''const CanMessage {val3} = {{
    "{val1}", 
    {val2}, 
    "{val3}", 
    {val4}, 
    {val5}, 
    {val6}, 
    {val7}, 
    {val8}, 
    {val9}, 
    "{val10}", 
    "{val11}" 
}};\n'''

if mode == "output":
    path = csv_file_paths[0]
else:
    path = csv_file_paths[1]

with open(path, newline='', encoding='utf-8') as f:
    with open("tools/task_complete.txt", 'w', encoding='utf-8') as out_f:
        reader = csv.reader(f)
        header = next(reader)  # skip header if it exists
        for i, row in enumerate(reader, start=1):
            # unpack row (expects 11 values in each row)
            val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11 = row
            
            # auto-generate a unique struct name from val1 or row index
            struct_name = val1.replace(" ", "_").replace("-", "_") + f"_{i}"
            if mode == "output":
                val3 = val3.replace(" ", "_").replace("-", "_")  # ensure val3 is a valid identifier
            
            # fill template
            if mode == "output":
                code = template.format(
                    name=val3,
                    val1=val1,
                    val2=val2,
                    val3=val3,
                    val4=val4,
                    val5=val5,
                    val6=val6,
                    val7=val7,
                    val8=val8,
                    val9=val9,
                    val10=val10,
                    val11=val11
                )
            else:
                code = template.format(
                    name=struct_name,
                    val1=val1,
                    val2=val2,
                    val3=val3,
                    val4=val4,
                    val5=val5,
                    val6=val6,
                    val7=val7,
                    val8=val8,
                    val9=val9,
                    val10=val10,
                    val11=val11
                )
            
            out_f.write(code)
