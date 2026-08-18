param([Parameter(Mandatory = $true)][string]$Path)

$stream = [System.IO.File]::OpenRead($Path)
$reader = [System.IO.BinaryReader]::new($stream)

try {
    if ([Text.Encoding]::ASCII.GetString($reader.ReadBytes(4)) -ne 'MDMP') {
        throw 'Not a Windows minidump.'
    }

    $null = $reader.ReadUInt32() # Version
    $streamCount = $reader.ReadUInt32()
    $directoryRva = $reader.ReadUInt32()

    $directories = @{}
    $stream.Position = $directoryRva
    for ($i = 0; $i -lt $streamCount; $i++) {
        $type = $reader.ReadUInt32()
        $size = $reader.ReadUInt32()
        $rva = $reader.ReadUInt32()
        $directories[$type] = [pscustomobject]@{ Size = $size; Rva = $rva }
    }

    $exceptionStreamType = [uint32]6
    if (-not $directories.ContainsKey($exceptionStreamType)) { throw 'Exception stream not found.' }
    $stream.Position = $directories[$exceptionStreamType].Rva
    $threadId = $reader.ReadUInt32()
    $null = $reader.ReadUInt32()
    $exceptionCode = $reader.ReadUInt32()
    $exceptionFlags = $reader.ReadUInt32()
    $null = $reader.ReadUInt64()
    $exceptionAddress = $reader.ReadUInt64()

    $modules = @()
    $moduleStreamType = [uint32]4
    if ($directories.ContainsKey($moduleStreamType)) {
        $stream.Position = $directories[$moduleStreamType].Rva
        $moduleCount = $reader.ReadUInt32()
        for ($i = 0; $i -lt $moduleCount; $i++) {
            $entry = $stream.Position
            $base = $reader.ReadUInt64()
            $size = $reader.ReadUInt32()
            $null = $reader.ReadUInt32()
            $null = $reader.ReadUInt32()
            $nameRva = $reader.ReadUInt32()

            $returnPosition = $stream.Position
            $stream.Position = $nameRva
            $byteLength = $reader.ReadUInt32()
            $name = [Text.Encoding]::Unicode.GetString($reader.ReadBytes($byteLength))
            $stream.Position = $returnPosition

            $modules += [pscustomobject]@{ Name = $name; Base = $base; Size = $size }
            $stream.Position = $entry + 108
        }
    }

    $faultModule = $modules | Where-Object {
        $exceptionAddress -ge $_.Base -and $exceptionAddress -lt ($_.Base + $_.Size)
    } | Select-Object -First 1

    [pscustomobject]@{
        Dump = $Path
        ThreadId = $threadId
        ExceptionCode = ('0x{0:X8}' -f $exceptionCode)
        ExceptionFlags = ('0x{0:X8}' -f $exceptionFlags)
        ExceptionAddress = ('0x{0:X16}' -f $exceptionAddress)
        FaultModule = $faultModule.Name
        ModuleOffset = if ($faultModule) { '0x{0:X}' -f ($exceptionAddress - $faultModule.Base) } else { $null }
        ModuleCount = $modules.Count
    }
}
finally {
    $reader.Dispose()
    $stream.Dispose()
}
