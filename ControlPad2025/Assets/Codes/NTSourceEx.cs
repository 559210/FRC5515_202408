using NetworkTablesSharp;
using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;


namespace NetworkTablesSharpEx
{
    public class Nt4Source
    {
        public readonly Nt4Client Client;

        private readonly Dictionary<string, object> _values = new Dictionary<string, object>();

        private readonly Dictionary<string, (string, Dictionary<string, object>)> _queuedPublishes = new Dictionary<string, (string, Dictionary<string, object>)>();

        private readonly Dictionary<string, Nt4SubscriptionOptions> _queuedSubscribes = new Dictionary<string, Nt4SubscriptionOptions>();

        public Nt4Source(string serverAddress = "127.0.0.1", string appName = "Nt4Unity", bool connectAutomatically = true, int port = 5810)
        {
            Client = new Nt4Client(appName, serverAddress, port, OnOpen, OnNewTopicData);
            if (connectAutomatically)
            {
                Client.Connect();
            }
        }

        public void PublishTopic(string topic, string type)
        {
            PublishTopic(topic, type, new Dictionary<string, object>());
        }

        public void PublishTopic(string topic, string type, Dictionary<string, object> properties)
        {
            if (Client.Connected())
            {
                Client.PublishTopic(topic, type, properties);
            }

            _queuedPublishes.TryAdd(topic, (type, properties));
        }

        public void PublishValue(string topic, object value)
        {
            if (Client.Connected())
            {
                Client.PublishValue(topic, value);
            }
        }

        public void Subscribe(string topic, double period = 0.1, bool all = false, bool topicsOnly = false, bool prefix = false)
        {
            if (Client.Connected())
            {
                Client.Subscribe(topic, period, all, topicsOnly, prefix);
            }

            _queuedSubscribes.TryAdd(topic, new Nt4SubscriptionOptions(period, all, topicsOnly, prefix));
        }

        public T GetValue<T>(string key)
        {
            if (_values.TryGetValue(key, out object value))
            {
                return ((TopicValue<T>)value).GetValue();
            }

            return default(T);
        }

        public T GetValue<T>(string key, long timestamp)
        {
            if (_values.TryGetValue(key, out object value))
            {
                return ((TopicValue<T>)value).GetValue(timestamp);
            }

            return default(T);
        }

        public bool Connected()
        {
            return Client.Connected();
        }

        public void Connect()
        {
            if (!Client.Connected())
            {
                Client.Connect();
            }
        }

        public void Disconnect()
        {
            if (Client.Connected())
            {
                Client.Disconnect();
            }
        }

        public long? GetServerTimeUs()
        {
            return Client.GetServerTimeUs();
        }

        private void OnOpen(object? sender, EventArgs e)
        {
            foreach (string key in _queuedPublishes.Keys)
            {
                var (type, properties) = _queuedPublishes[key];
                Client.PublishTopic(key, type, properties);
            }

            foreach (string key2 in _queuedSubscribes.Keys)
            {
                Client.Subscribe(key2, _queuedSubscribes[key2]);
            }
        }

        private void OnNewTopicData(Nt4Topic topic, long timestamp, object value)
        {
            try
            {
                switch (topic.Type)
                {
                    case "string":
                        AddValue(topic.Name, timestamp, value.ToString());
                        break;
                    case "boolean":
                        AddValue(topic.Name, timestamp, Convert.ToBoolean(value));
                        break;
                    case "int":
                        AddValue(topic.Name, timestamp, Convert.ToInt64(value));
                        break;
                    case "double":
                        AddValue(topic.Name, timestamp, Convert.ToDouble(value));
                        break;
                    case "string[]":
                        AddValue(topic.Name, timestamp, ((object[])value).Cast<string>().ToArray());
                        break;
                    case "boolean[]":
                        AddValue(topic.Name, timestamp, ((object[])value).Cast<bool>().ToArray());
                        break;
                    case "int[]":
                        {
                            object[] val = (object[])value;
                            long[] data = new long[val.Length];
                            for (int i = 0; i < val.Length; ++i)
                            {
                                //Debug.LogError(val[i].GetType());
                                //Type type = val[i].GetType();

                                var v = val[i];
                                if (v is System.SByte)
                                {
                                    data[i] = (long)(System.SByte)v;
                                }
                                else if (v is System.Byte)
                                {
                                    data[i] = (long)(System.Byte)v;
                                }
                                else if (v is Int16)
                                {
                                    data[i] = (long)(System.Int16)v;
                                }
                                else if (v is UInt16)
                                {
                                    data[i] = (long)(System.UInt16)v;
                                }
                                else if (v is Int32)
                                {
                                    data[i] = (long)(System.Int32)v;
                                }
                                else if (v is UInt32)
                                {
                                    data[i] = (long)(System.UInt32)v;
                                }
                                else if (v is Int64)
                                {
                                    data[i] = (long)(System.Int64)v;
                                }
                                else if (v is UInt64)
                                {
                                    data[i] = (long)(System.UInt64)v;
                                }
                                else
                                {
                                    data[i] = long.Parse(val[i].ToString());
                                }
                            }
                            AddValue(topic.Name, timestamp, data);
                        }
                    
                        break;
                    case "double[]":
                        AddValue(topic.Name, timestamp, ((object[])value).Cast<double>().ToArray());
                        break;
                    default:
                        throw new ArgumentException("Unknown type " + topic.Type);
                }
            }
            catch (Exception e)
            {
                UnityEngine.Debug.LogError(e);
            }
        }

        private void AddValue<T>(string key, long timestamp, T value)
        {
            if (!_values.ContainsKey(key))
            {
                _values[key] = new TopicValue<T>();
            }

            ((TopicValue<T>)_values[key]).AddValue(timestamp, value);
        }
    }


}

