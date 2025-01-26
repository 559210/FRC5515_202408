# NetworkTablesSharp

A bare-bones implementation of NT4 (4.0) for C#. Designed for unity but does not rely on it.

If you are looking to use this package for unity, consider following [this guide](https://www.jacksondunstan.com/articles/2986) so that log messages appear in the console.

> Disclaimer: This package has not been thourogly tested, and may have issues. This package may or may not be updated and supported in the future.

## Installation

> Note: Unity users will need to install [NuGetForUnity](https://github.com/GlitchEnzo/NuGetForUnity)

Install this NuGet Package: [NetworkTablesSharp](https://www.nuget.org/packages/NetworkTablesSharp)

## Usage

For convinience and usability, `Nt4Source` is provided. It provides a user-friendly wrapper around the client with convinience features such as queing subscriptions/topic publishes to ensure things work even after a reconnect. It also stores all previous values.

**Connecting**

```cs
// Default values shown here
Nt4Source Source = new Nt4Source(string serverAddress = "127.0.0.1", string appName = "Nt4Unity", bool connectAutomatically = true, int port = 5810);

// Not needed if you leave connectAutomatically as true
Source.Connect();
Source.Disconnect();
```

**Subscribing and retrieving data**

```cs
Source.Subscribe("Key");

string latestValue = Source.GetValue<string>("Key");
string specificValue = Source.GetValue<string>("Key", Source.GetServerTimeUs());
```

**Publishing data**

> Note: Publishing data has not been properly tested.

```cs
Source.PublishTopic("key", "type" /* ex. string, int */);
Source.PublishValue("key", "value")
```

If you prefer, you can also use the Nt4Client class directly. The API Is documented, look at the file to see how to use it.

## Credits

- Modeled after the NT4 Code from [AdvantageScope](https://github.com/MechanicalAdvantage/AdvantageScope)
- [Newtonsoft.Json](https://www.nuget.org/packages/Newtonsoft.Json)
- [WebSocketSharp](https://www.nuget.org/packages/WebSocketSharp)
- [MessagePack](https://www.nuget.org/packages/MessagePack)
